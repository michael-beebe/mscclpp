// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#pragma once

#include <chrono>
#include <comms/torchcomms/TorchWork.hpp>
#include <mscclpp/gpu.hpp>
#include <mutex>
#include <optional>
#include <vector>

namespace torch::comms {

/// GPU event pool — reuses CUDA/HIP events to avoid alloc/free overhead.
///
/// Thread-safe. Owned by TorchCommMSCCLPP and borrowed by TorchWorkMSCCLPP.
/// Events are created with cudaEventDisableTiming (no timing overhead) since
/// we only use them for stream synchronization.
class MscclppGpuEventPool {
 public:
  explicit MscclppGpuEventPool(size_t max_size = 256);
  ~MscclppGpuEventPool();

  MscclppGpuEventPool(const MscclppGpuEventPool&) = delete;
  MscclppGpuEventPool& operator=(const MscclppGpuEventPool&) = delete;

  /// Acquire an event from the pool (or allocate a new one if empty).
  cudaEvent_t acquire();

  /// Return an event to the pool. If the pool is full, destroys the event.
  void release(cudaEvent_t event);

 private:
  std::vector<cudaEvent_t> available_;
  std::mutex mutex_;
  size_t max_size_;
};

/// GPU event-based async work handle for MSCCL++ operations.
///
/// Records a single end-of-collective event on the operation stream. wait()
/// inserts a cudaStreamWaitEvent on the caller's current stream (GPU-side,
/// no CPU blocking). checkStatus() polls the end event and enforces a
/// CPU-side wall-clock timeout (the start reference is captured at
/// construction, no GPU sync required).
///
/// We deliberately do NOT record a start event — every cudaEventRecord
/// is 2-5us of driver time, and the start event was only consulted by
/// checkStatus() for timeout-window measurement. Wall-clock from
/// construction is a good enough timeout reference and avoids per-call
/// driver overhead, which matters in FSDP2 where each step issues
/// dozens of collectives.
class TorchWorkMSCCLPP : public TorchWork {
 public:
  TorchWorkMSCCLPP(cudaStream_t op_stream, int device_index, std::chrono::milliseconds timeout_ms,
                   std::shared_ptr<MscclppGpuEventPool> event_pool);
  ~TorchWorkMSCCLPP() override;

  TorchWorkMSCCLPP(const TorchWorkMSCCLPP&) = delete;
  TorchWorkMSCCLPP& operator=(const TorchWorkMSCCLPP&) = delete;

  void wait() override;
  std::chrono::milliseconds getTimeout() const override { return timeout_ms_; }

  /// No-op kept for backward-compat with existing call sites.
  void recordStart() {}

  /// Record end event on op_stream_ after launching the collective.
  void recordEnd();

 private:
  WorkStatus checkStatus();

  cudaEvent_t end_event_;
  cudaStream_t op_stream_;  // not owned
  int device_index_;
  std::chrono::milliseconds timeout_ms_;
  std::shared_ptr<MscclppGpuEventPool> event_pool_;
  std::chrono::steady_clock::time_point construct_time_;
};

}  // namespace torch::comms
