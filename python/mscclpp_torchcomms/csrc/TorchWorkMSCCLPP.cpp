// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#include "TorchWorkMSCCLPP.hpp"

#include <c10/cuda/CUDAStream.h>

#include <mscclpp/gpu_utils.hpp>
#include <stdexcept>

namespace torch::comms {

// --- MscclppGpuEventPool ---
//
// CUDA/HIP event allocation is expensive (~5-10us per cudaEventCreate).
// Since every collective call needs 2 events (start + end), and training
// loops run thousands of collectives, we pool and reuse events to avoid
// that overhead. The pool is thread-safe and shared across all
// TorchWorkMSCCLPP instances from the same communicator.

MscclppGpuEventPool::MscclppGpuEventPool(size_t max_size) : max_size_(max_size) {}

MscclppGpuEventPool::~MscclppGpuEventPool() {
  std::lock_guard<std::mutex> lock(mutex_);
  for (auto event : available_) {
    cudaEventDestroy(event);
  }
  available_.clear();
}

// Returns a recycled event if one is available, otherwise allocates a new one.
// Events use cudaEventDisableTiming because we only need them for stream
// synchronization (cudaStreamWaitEvent), not for measuring elapsed time.
cudaEvent_t MscclppGpuEventPool::acquire() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!available_.empty()) {
    cudaEvent_t event = available_.back();
    available_.pop_back();
    return event;
  }
  cudaEvent_t event;
  MSCCLPP_CUDATHROW(cudaEventCreateWithFlags(&event, cudaEventDisableTiming));
  return event;
}

// Returns an event to the pool for reuse. If the pool is already at capacity,
// the event is destroyed instead to avoid unbounded memory growth.
void MscclppGpuEventPool::release(cudaEvent_t event) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (available_.size() < max_size_) {
    available_.push_back(event);
  } else {
    cudaEventDestroy(event);
  }
}

// --- TorchWorkMSCCLPP ---
//
// Every TorchComms collective must return a TorchWork handle so the caller
// can track completion. TorchWorkMSCCLPP uses GPU events to do this without
// CPU blocking:
//
//   1. Before launching the collective kernel: recordStart() records start_event_
//   2. After launching: recordEnd() records end_event_
//   3. wait(): makes the caller's PyTorch stream wait on end_event_ via
//      cudaStreamWaitEvent — this is purely GPU-side stream ordering,
//      the CPU returns immediately
//   4. checkStatus(): polls events for completion/timeout detection
//
// This matches the TorchWorkNCCL pattern in the torchcomms NCCL backend.

// Acquires one event from the pool. Returned to the pool in the destructor.
TorchWorkMSCCLPP::TorchWorkMSCCLPP(cudaStream_t op_stream, int device_index, std::chrono::milliseconds timeout_ms,
                                   std::shared_ptr<MscclppGpuEventPool> event_pool)
    : op_stream_(op_stream),
      device_index_(device_index),
      timeout_ms_(timeout_ms),
      event_pool_(std::move(event_pool)),
      construct_time_(std::chrono::steady_clock::now()) {
  end_event_ = event_pool_->acquire();
}

TorchWorkMSCCLPP::~TorchWorkMSCCLPP() { event_pool_->release(end_event_); }

// Records a GPU event on the operation stream AFTER the collective kernel
// is launched. wait() and checkStatus() use this event to determine when
// the collective has finished.
void TorchWorkMSCCLPP::recordEnd() { MSCCLPP_CUDATHROW(cudaEventRecord(end_event_, op_stream_)); }

// Polls the end event without blocking, enforces wall-clock timeout.
TorchWork::WorkStatus TorchWorkMSCCLPP::checkStatus() {
  if (status() == WorkStatus::COMPLETED || status() == WorkStatus::ERROR || status() == WorkStatus::TIMEDOUT) {
    return status();
  }
  cudaError_t end_status = cudaEventQuery(end_event_);
  if (end_status == cudaSuccess) {
    setStatus(WorkStatus::COMPLETED);
  } else if (end_status == cudaErrorNotReady) {
    setStatus(WorkStatus::INPROGRESS);
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() -
                                                                         construct_time_);
    if (elapsed > timeout_ms_) {
      setStatus(WorkStatus::TIMEDOUT);
    }
  } else {
    setStatus(WorkStatus::ERROR);
  }
  return status();
}

// Called by the user (or TorchComm wrapper) to synchronize on the collective.
// This does NOT block the CPU — it inserts a dependency edge on the GPU:
// the caller's current PyTorch CUDA stream will wait for end_event_ before
// executing any subsequent kernels. This is the same pattern NCCL uses.
void TorchWorkMSCCLPP::wait() {
  WorkStatus current = checkStatus();
  if (current == WorkStatus::COMPLETED || current == WorkStatus::ERROR || current == WorkStatus::TIMEDOUT) {
    return;
  }

  // GPU-side wait: make the caller's current stream wait on end_event_.
  // No CPU blocking — just stream ordering.
  cudaStream_t current_stream = at::cuda::getCurrentCUDAStream(device_index_).stream();
  MSCCLPP_CUDATHROW(cudaStreamWaitEvent(current_stream, end_event_, 0));
  setStatus(WorkStatus::COMPLETED);
}

}  // namespace torch::comms
