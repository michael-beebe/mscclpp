// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

// Dynamic-loader entry point for the upstream torchcomms NCCL backend.
//
// Why this file exists separately from the upstream torchcomms tree:
//
// The torchcomms TorchCommFactory dlopen-based loader path
// (TorchCommFactory::create_generic_backend in TorchCommFactory.cpp) wraps the
// raw pointer returned by `loader.new_comm()` in a
// `std::shared_ptr<TorchCommBackend>(rawBackendPtr, deleter)`. The
// `enable_shared_from_this<Y>` mechanism only initializes its internal
// weak_ptr when the shared_ptr is constructed from a pointer to the *derived*
// type `Y`. Constructing `shared_ptr<TorchCommBackend>` from a pointer
// statically typed as `TorchCommBackend*` skips that machinery, so when
// `TorchCommNCCL::createWork()` later calls `shared_from_this()` it throws
// `std::bad_weak_ptr` (the very first all_reduce crashes).
//
// To work around this without patching torchcomms, this loader keeps a
// keep-alive `shared_ptr<TorchCommNCCL>` (created via `std::make_shared` so
// the weak_ptr is set up correctly) alive in a static map keyed by the raw
// `TorchCommBackend*` we hand back. The factory still wraps our pointer in
// its own `shared_ptr<TorchCommBackend>` for ownership semantics, and
// destroy_comm_impl drops the keep-alive entry — but as long as the entry
// lives, `shared_from_this()` inside the NCCL backend successfully
// constructs a new shared_ptr that aliases our keep-alive one.

#include <comms/torchcomms/TorchCommBackend.hpp>
#include <comms/torchcomms/nccl/TorchCommNCCL.hpp>

#include <memory>
#include <mutex>
#include <unordered_map>

namespace {

std::mutex& keepaliveMutex() {
  static std::mutex m;
  return m;
}

std::unordered_map<torch::comms::TorchCommBackend*, std::shared_ptr<torch::comms::TorchCommNCCL>>&
keepaliveMap() {
  static std::unordered_map<torch::comms::TorchCommBackend*, std::shared_ptr<torch::comms::TorchCommNCCL>>
      m;
  return m;
}

torch::comms::TorchCommBackend* new_comm_impl() {
  auto sp = std::make_shared<torch::comms::TorchCommNCCL>();
  auto* base = static_cast<torch::comms::TorchCommBackend*>(sp.get());
  {
    std::lock_guard<std::mutex> guard(keepaliveMutex());
    keepaliveMap().emplace(base, std::move(sp));
  }
  return base;
}

void destroy_comm_impl(torch::comms::TorchCommBackend* comm) {
  std::lock_guard<std::mutex> guard(keepaliveMutex());
  auto it = keepaliveMap().find(comm);
  if (it != keepaliveMap().end()) {
    keepaliveMap().erase(it);
  } else {
    delete comm;
  }
}

const char* get_supported_version_impl() {
  return torch::comms::TORCHCOMM_BACKEND_ABI_VERSION;
}

}  // namespace

extern "C" __attribute__((visibility("default"))) torch::comms::DynamicLoaderInterface
create_dynamic_loader_nccl() {
  return torch::comms::DynamicLoaderInterface{
      .new_comm = new_comm_impl,
      .destroy_comm = destroy_comm_impl,
      .get_supported_version = get_supported_version_impl,
  };
}
