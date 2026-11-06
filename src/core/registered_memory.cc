// Copyright (c) Microsoft Corporation.
// Licensed under the MIT license.

#include "registered_memory.hpp"

#include <sys/syscall.h>
#include <unistd.h>

#include <algorithm>
#include <cstring>
#include <mscclpp/gpu_utils.hpp>
#include <unordered_map>

#include "api.h"
#include "context.hpp"
#include "logger.hpp"
#include "serialization.hpp"
#include "unix_socket.hpp"
#include "utils_internal.hpp"

#define MSCCLPP_CULOG_WARN(cmd)                                         \
  do {                                                                  \
    CUresult err = cmd;                                                 \
    if (err != CUDA_SUCCESS) {                                          \
      const char* errStr;                                               \
      if (cuGetErrorString(err, &errStr) != CUDA_SUCCESS) {             \
        errStr = "failed to get error string";                          \
      }                                                                 \
      WARN(mscclpp::GPU, "Call to " #cmd " failed, error is ", errStr); \
    }                                                                   \
  } while (false)

namespace mscclpp {

static void checkMemoryMetadataSize(std::vector<char>::const_iterator it, std::vector<char>::const_iterator end,
                                    size_t size) {
  if (end - it < static_cast<std::ptrdiff_t>(size)) {
    throw Error("Truncated registered memory metadata", ErrorCode::InvalidUsage);
  }
}

RegisteredMemory::Impl::Impl(void* data, size_t size, TransportFlags transports,
                             [[maybe_unused]] Context::Impl& contextImpl)
    : data(data),
      originalDataPtr(data),
      size(size),
      hostHash(getHostHash()),
      pidHash(getPidHash()),
      transports(transports) {
#if !defined(MSCCLPP_USE_IB)
  if ((transports & AllIBTransports).any()) {
    throw Error("IB support is disabled", ErrorCode::InvalidUsage);
  }
#endif
#if !defined(MSCCLPP_USE_MRC)
  if ((transports & AllMrcTransports).any()) {
    throw Error("Native MRC support is disabled", ErrorCode::InvalidUsage);
  }
#endif
  if (transports.has(Transport::CudaIpc)) {
    CudaDeviceGuard deviceGuard(detail::gpuIdFromAddress(data));

    localGpuIpcMemHandle = GpuIpcMemHandle::create(reinterpret_cast<CUdeviceptr>(data));
    TransportInfo transportInfo;
    transportInfo.transport = Transport::CudaIpc;
    transportInfo.gpuIpcMemHandle = *localGpuIpcMemHandle;
    this->transportInfos.emplace_back(transportInfo);
  }
#if defined(MSCCLPP_USE_IB)
  if ((transports & AllIBTransports).any()) {
    auto addIb = [&](Transport ibTransport) {
      TransportInfo transportInfo;
      transportInfo.transport = ibTransport;
      this->ibMrMap[ibTransport] = contextImpl.getIbContext(ibTransport)->registerMr(data, size);
      transportInfo.ibMr = this->ibMrMap[ibTransport].get();
      transportInfo.ibLocal = true;
      transportInfo.ibMrInfo = this->ibMrMap[ibTransport]->getInfo();
      this->transportInfos.push_back(transportInfo);
      INFO(NET, "IB mr for address ", data, " with size ", size, " is registered");
    };
    if (transports.has(Transport::IB0)) addIb(Transport::IB0);
    if (transports.has(Transport::IB1)) addIb(Transport::IB1);
    if (transports.has(Transport::IB2)) addIb(Transport::IB2);
    if (transports.has(Transport::IB3)) addIb(Transport::IB3);
    if (transports.has(Transport::IB4)) addIb(Transport::IB4);
    if (transports.has(Transport::IB5)) addIb(Transport::IB5);
    if (transports.has(Transport::IB6)) addIb(Transport::IB6);
    if (transports.has(Transport::IB7)) addIb(Transport::IB7);
  }
#endif
#if defined(MSCCLPP_USE_MRC)
  for (Transport transport : {Transport::MRC0, Transport::MRC1, Transport::MRC2, Transport::MRC3}) {
    if (!transports.has(transport)) continue;
    auto mr = contextImpl.getMrcContext(transport)->registerMr(data, size);
    TransportInfo info{};
    info.transport = transport;
    info.mrcLocal = true;
    info.mrcMr = mr.get();
    info.mrcMrInfo = mr->getInfo();
    mrcMrMap.emplace(transport, std::move(mr));
    transportInfos.push_back(info);
    INFO(NET, "Native MRC mr for address ", data, " with size ", size, " is registered");
  }
#endif
}

MSCCLPP_API_CPP RegisteredMemory::RegisteredMemory(std::shared_ptr<Impl> pimpl) : pimpl_(pimpl) {}

MSCCLPP_API_CPP RegisteredMemory::~RegisteredMemory() = default;

MSCCLPP_API_CPP void* RegisteredMemory::data() const { return pimpl_->data; }

MSCCLPP_API_CPP void* RegisteredMemory::originalDataPtr() const { return pimpl_->originalDataPtr; }

MSCCLPP_API_CPP size_t RegisteredMemory::size() const { return pimpl_->size; }

MSCCLPP_API_CPP TransportFlags RegisteredMemory::transports() const { return pimpl_->transports; }

MSCCLPP_API_CPP std::vector<char> RegisteredMemory::serialize() const {
  std::vector<char> result;
  detail::serialize(result, pimpl_->originalDataPtr);
  detail::serialize(result, pimpl_->size);
  detail::serialize(result, pimpl_->hostHash);
  detail::serialize(result, pimpl_->pidHash);
  detail::serialize(result, pimpl_->transports);
  if (pimpl_->transportInfos.size() > static_cast<size_t>(std::numeric_limits<int8_t>::max())) {
    throw Error("Too many transport info entries", ErrorCode::InternalError);
  }
  int8_t transportCount = pimpl_->transportInfos.size();
  detail::serialize(result, transportCount);
  for (auto& entry : pimpl_->transportInfos) {
    detail::serialize(result, entry.transport);
    if (entry.transport == Transport::CudaIpc) {
      if (entry.gpuIpcMemHandle.typeFlags == GpuIpcMemHandle::Type::None) {
        THROW(GPU, Error, ErrorCode::InternalError, "GpuIpcMemHandle type is None");
      }
      detail::serialize(result, entry.gpuIpcMemHandle);
    } else if (AllIBTransports.has(entry.transport)) {
#if defined(MSCCLPP_USE_IB)
      detail::serialize(result, entry.ibMrInfo);
#else
      throw Error("IB support is disabled", ErrorCode::InvalidUsage);
#endif
    } else if (AllMrcTransports.has(entry.transport)) {
#if defined(MSCCLPP_USE_MRC)
      detail::serialize(result, entry.mrcMrInfo);
#else
      throw Error("Native MRC support is disabled", ErrorCode::InvalidUsage);
#endif
    } else {
      THROW(GPU, Error, ErrorCode::InternalError, "Unknown transport");
    }
  }
  return result;
}

MSCCLPP_API_CPP RegisteredMemory RegisteredMemory::deserialize(const std::vector<char>& data) {
  return RegisteredMemory(std::make_shared<Impl>(data));
}

RegisteredMemory::Impl::Impl(const std::vector<char>::const_iterator& begin,
                             const std::vector<char>::const_iterator& end) {
  auto it = begin;
  checkMemoryMetadataSize(it, end,
                          sizeof(originalDataPtr) + sizeof(size) + sizeof(hostHash) + sizeof(pidHash) +
                              sizeof(transports) + sizeof(int8_t));
  it = detail::deserialize(it, this->originalDataPtr);
  it = detail::deserialize(it, this->size);
  it = detail::deserialize(it, this->hostHash);
  it = detail::deserialize(it, this->pidHash);
  it = detail::deserialize(it, this->transports);
#if !defined(MSCCLPP_USE_IB)
  if ((this->transports & AllIBTransports).any()) {
    throw Error("IB support is disabled", ErrorCode::InvalidUsage);
  }
#endif
#if !defined(MSCCLPP_USE_MRC)
  if ((this->transports & AllMrcTransports).any()) {
    throw Error("Native MRC support is disabled", ErrorCode::InvalidUsage);
  }
#endif
  int8_t transportCount;
  it = detail::deserialize(it, transportCount);
  if (transportCount < 0) throw Error("Invalid transport record count", ErrorCode::InvalidUsage);
  TransportFlags seenMrc;
  for (int i = 0; i < transportCount; ++i) {
    TransportInfo transportInfo{};
    checkMemoryMetadataSize(it, end, sizeof(transportInfo.transport));
    it = detail::deserialize(it, transportInfo.transport);
    if (static_cast<unsigned int>(transportInfo.transport) >= static_cast<unsigned int>(Transport::NumTransports)) {
      throw Error("Invalid memory transport identity", ErrorCode::InvalidUsage);
    }
    if (transportInfo.transport == Transport::CudaIpc) {
      checkMemoryMetadataSize(it, end, sizeof(transportInfo.gpuIpcMemHandle));
      it = detail::deserialize(it, transportInfo.gpuIpcMemHandle);
    } else if (AllIBTransports.has(transportInfo.transport)) {
#if defined(MSCCLPP_USE_IB)
      checkMemoryMetadataSize(it, end, sizeof(transportInfo.ibMrInfo));
      it = detail::deserialize(it, transportInfo.ibMrInfo);
      transportInfo.ibLocal = false;
#else
      throw Error("IB support is disabled", ErrorCode::InvalidUsage);
#endif
    } else if (AllMrcTransports.has(transportInfo.transport)) {
#if defined(MSCCLPP_USE_MRC)
      if (!this->transports.has(transportInfo.transport) || seenMrc.has(transportInfo.transport)) {
        throw Error("Unexpected or duplicate native MRC memory record", ErrorCode::InvalidUsage);
      }
      checkMemoryMetadataSize(it, end, sizeof(transportInfo.mrcMrInfo));
      it = detail::deserialize(it, transportInfo.mrcMrInfo);
      if (this->size == 0 || transportInfo.mrcMrInfo.addr == 0 ||
          transportInfo.mrcMrInfo.addr != reinterpret_cast<uintptr_t>(this->originalDataPtr) ||
          this->size > std::numeric_limits<uint64_t>::max() - transportInfo.mrcMrInfo.addr) {
        throw Error("Invalid native MRC memory range", ErrorCode::InvalidUsage);
      }
      transportInfo.mrcLocal = false;
      seenMrc |= transportInfo.transport;
#else
      throw Error("Native MRC support is disabled", ErrorCode::InvalidUsage);
#endif
    } else {
      throw Error("Unknown transport", ErrorCode::InternalError);
    }
    this->transportInfos.emplace_back(transportInfo);
  }
  if (seenMrc != (this->transports & AllMrcTransports)) {
    throw Error("Missing native MRC memory records", ErrorCode::InvalidUsage);
  }
  if (it != end) {
    throw Error("Serialization failed", ErrorCode::InternalError);
  }

  // Next decide how to set this->data
  this->data = nullptr;
  if (getHostHash() == this->hostHash && getPidHash() == this->pidHash) {
    // The memory is local to the process, so originalDataPtr is valid as is
    this->data = this->originalDataPtr;
    if (transports.has(Transport::CudaIpc)) {
      auto entry = getTransportInfo(Transport::CudaIpc);
      if ((entry.gpuIpcMemHandle.typeFlags & GpuIpcMemHandle::Type::RuntimeIpc) == 0) {
        // Query which device owns this memory
        int gpuId = detail::gpuIdFromAddress(this->data);
        int currentDevice = -1;
        MSCCLPP_CUDATHROW(cudaGetDevice(&currentDevice));

        // Only set access if we're on a different device than where memory was allocated
        if (gpuId != currentDevice) {
          detail::setReadWriteMemoryAccess(this->data, entry.gpuIpcMemHandle.baseSize);
        }
      }
    }
  } else if (transports.has(Transport::CudaIpc)) {
    // When IPC and RDMA metadata coexist, try IPC first and retain RDMA on import failure.
    auto entry = getTransportInfo(Transport::CudaIpc);
    bool hasIB = (transports & AllIBTransports).any();
    bool hasMrc = (transports & AllMrcTransports).any();
    try {
      auto gpuIpcMem = GpuIpcMem::create(entry.gpuIpcMemHandle);
      this->remoteMemMap = gpuIpcMem->map();
      this->data = this->remoteMemMap.get();
    } catch (const BaseError& e) {
      if (!hasIB && !hasMrc) {
        throw;
      }
      bool isSameHost = (getHostHash() == this->hostHash);
      std::string fallback = hasIB ? "IB" : "native MRC";
      if (isSameHost) {
        WARN(GPU, "CudaIpc import failed on same host, falling back to ", fallback, " transport: ", e.what());
      } else {
        INFO(GPU, "CudaIpc import failed on remote host, falling back to ", fallback, " transport: ", e.what());
      }
    }
  }
  if (this->data != nullptr) {
    INFO(GPU, "Opened CUDA IPC handle at pointer ", this->data);
  }
}

RegisteredMemory::Impl::Impl(const std::vector<char>& serialization)
    : Impl(serialization.begin(), serialization.end()) {}

const TransportInfo& RegisteredMemory::Impl::getTransportInfo(Transport transport) const {
  for (auto& entry : transportInfos) {
    if (entry.transport == transport) {
      return entry;
    }
  }
  throw Error("Transport data not found", ErrorCode::InternalError);
}

}  // namespace mscclpp
