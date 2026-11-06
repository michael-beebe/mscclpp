// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#include "endpoint.hpp"

#include <algorithm>
#include <mscclpp/env.hpp>

#include "api.h"
#include "context.hpp"
#include "ib.hpp"
#include "logger.hpp"
#include "registered_memory.hpp"
#include "serialization.hpp"
#include "socket.h"
#include "utils_internal.hpp"

namespace mscclpp {

#if defined(MSCCLPP_USE_MRC)
static void validateMrcConfig(const EndpointConfig& config) {
  const auto& mrc = config.mrc;
  if (mrc.port <= 0 || mrc.port > 255 || mrc.gidIndex < 0 || mrc.maxCqSize <= 0 || mrc.maxSendWr <= 0 ||
      mrc.maxRecvWr < 0 || config.maxWriteQueueSize <= 0) {
    throw Error("Invalid native MRC endpoint configuration", ErrorCode::InvalidUsage);
  }
}
#endif

Endpoint::Impl::Impl(const EndpointConfig& config, Context::Impl& contextImpl)
    : config_(config), hostHash_(getHostHash()), pidHash_(getPidHash()) {
#if !defined(MSCCLPP_USE_IB)
  if (AllIBTransports.has(config_.transport)) {
    throw Error("IB support is disabled", ErrorCode::InvalidUsage);
  }
#endif
#if !defined(MSCCLPP_USE_MRC)
  if (AllMrcTransports.has(config_.transport)) {
    throw Error("Native MRC support is disabled", ErrorCode::InvalidUsage);
  }
#endif
  if (config_.device.type == DeviceType::GPU && config_.device.id < 0) {
    MSCCLPP_CUDATHROW(cudaGetDevice(&(config_.device.id)));
  }
  if (AllMrcTransports.has(config_.transport)) {
#if defined(MSCCLPP_USE_MRC)
    if (config_.maxWriteQueueSize <= 0) config_.maxWriteQueueSize = config_.mrc.maxCqSize;
    if (config_.mrc.gidIndex < 0) config_.mrc.gidIndex = env()->ibGidIndex;
    validateMrcConfig(config_);
    mrcLocal_ = true;
    mrcQp_ = contextImpl.getMrcContext(config_.transport)
                 ->createQp(config_.mrc.port, config_.mrc.gidIndex, config_.mrc.evProfileId, config_.mrc.maxCqSize,
                            config_.mrc.maxSendWr, config_.mrc.maxRecvWr);
    mrcQpInfo_ = mrcQp_->getInfo();
#else
    throw Error("Native MRC support is disabled", ErrorCode::InvalidUsage);
#endif
  } else if (AllIBTransports.has(config_.transport)) {
#if defined(MSCCLPP_USE_IB)
    ibLocal_ = true;
    if (config_.maxWriteQueueSize <= 0) {
      config_.maxWriteQueueSize = config_.ib.maxCqSize;
    }

    // Determine if we should use no-atomics mode
    ibNoAtomic_ = false;
    if (config_.ib.mode == EndpointConfig::Ib::Mode::HostNoAtomic) {
      ibNoAtomic_ = true;
    } else if (config_.ib.mode == EndpointConfig::Ib::Mode::Default) {
      // Use environment variable when mode is Default
      ibNoAtomic_ = (env()->ibvMode == "host-no-atomic");
    }

    // If mode is Host (or Default resolved to host), check if atomics are supported
    if (!ibNoAtomic_) {
      IbCtx* ibCtx = contextImpl.getIbContext(config_.transport);
      if (!ibCtx->supportsRdmaAtomics()) {
        WARN(NET, "IB device ", ibCtx->getDevName(),
             " does not support RDMA atomics. Falling back to write-with-immediate mode (HostNoAtomic).");
        ibNoAtomic_ = true;
      }
    }

    // Resolve GID index: explicit value (>= 0) takes priority, otherwise use env
    if (config_.ib.gidIndex < 0) {
      config_.ib.gidIndex = env()->ibGidIndex;
    }

    int maxRecvWr = ibNoAtomic_ ? config_.ib.maxRecvWr : 0;

    ibQp_ = contextImpl.getIbContext(config_.transport)
                ->createQp(config_.ib.port, config_.ib.gidIndex, config_.ib.maxCqSize, config_.ib.maxCqPollNum,
                           config_.ib.maxSendWr, maxRecvWr, config_.ib.maxWrPerSend, ibNoAtomic_);
    ibQpInfo_ = ibQp_->getInfo();
#else
    throw Error("IB support is disabled", ErrorCode::InvalidUsage);
#endif
  } else if (config_.transport == Transport::Ethernet) {
    // Configuring Ethernet Interfaces
    abortFlag_ = 0;
    int ret = FindInterfaces(netIfName_, &socketAddress_, MAX_IF_NAME_SIZE, 1);
    if (ret <= 0) throw Error("Failed to find network interfaces", ErrorCode::InternalError);

    // Starting Server Socket
    socket_ = std::make_unique<Socket>(&socketAddress_, MSCCLPP_SOCKET_MAGIC, SocketTypeBootstrap, abortFlag_);
    socket_->bindAndListen();
    socketAddress_ = socket_->getAddr();
  }
}

Endpoint::Impl::Impl(const std::vector<char>& serialization) {
  if (serialization.size() < sizeof(config_) + sizeof(hostHash_) + sizeof(pidHash_)) {
    throw Error("Truncated endpoint metadata", ErrorCode::InvalidUsage);
  }
  auto it = serialization.begin();
  it = detail::deserialize(it, config_);
  it = detail::deserialize(it, hostHash_);
  it = detail::deserialize(it, pidHash_);
  if (AllMrcTransports.has(config_.transport)) {
#if defined(MSCCLPP_USE_MRC)
    validateMrcConfig(config_);
    mrcLocal_ = false;
    if (serialization.end() - it < static_cast<std::ptrdiff_t>(sizeof(mrcQpInfo_))) {
      throw Error("Truncated native MRC endpoint metadata", ErrorCode::InvalidUsage);
    }
    it = detail::deserialize(it, mrcQpInfo_);
#else
    throw Error("Native MRC support is disabled", ErrorCode::InvalidUsage);
#endif
  } else if (AllIBTransports.has(config_.transport)) {
#if defined(MSCCLPP_USE_IB)
    ibLocal_ = false;
    if (serialization.end() - it < static_cast<std::ptrdiff_t>(sizeof(ibQpInfo_) + sizeof(ibNoAtomic_))) {
      throw Error("Truncated IB endpoint metadata", ErrorCode::InvalidUsage);
    }
    it = detail::deserialize(it, ibQpInfo_);
    it = detail::deserialize(it, ibNoAtomic_);
#else
    throw Error("IB support is disabled", ErrorCode::InvalidUsage);
#endif
  } else if (config_.transport == Transport::Ethernet) {
    if (serialization.end() - it < static_cast<std::ptrdiff_t>(sizeof(socketAddress_))) {
      throw Error("Truncated Ethernet endpoint metadata", ErrorCode::InvalidUsage);
    }
    it = detail::deserialize(it, socketAddress_);
  }
  if (it != serialization.end()) {
    throw Error("Endpoint deserialization failed", ErrorCode::Aborted);
  }
}

MSCCLPP_API_CPP Endpoint::Endpoint(std::shared_ptr<Endpoint::Impl> pimpl) : pimpl_(pimpl) {}

MSCCLPP_API_CPP const EndpointConfig& Endpoint::config() const { return pimpl_->config_; }

MSCCLPP_API_CPP Transport Endpoint::transport() const { return pimpl_->config_.transport; }

MSCCLPP_API_CPP const Device& Endpoint::device() const { return pimpl_->config_.device; }

MSCCLPP_API_CPP uint64_t Endpoint::hostHash() const { return pimpl_->hostHash_; }

MSCCLPP_API_CPP uint64_t Endpoint::pidHash() const { return pimpl_->pidHash_; }

MSCCLPP_API_CPP int Endpoint::maxWriteQueueSize() const { return pimpl_->config_.maxWriteQueueSize; }

MSCCLPP_API_CPP std::vector<char> Endpoint::serialize() const {
  std::vector<char> data;
  detail::serialize(data, pimpl_->config_);
  detail::serialize(data, pimpl_->hostHash_);
  detail::serialize(data, pimpl_->pidHash_);
  if (AllMrcTransports.has(pimpl_->config_.transport)) {
#if defined(MSCCLPP_USE_MRC)
    detail::serialize(data, pimpl_->mrcQpInfo_);
#else
    throw Error("Native MRC support is disabled", ErrorCode::InvalidUsage);
#endif
  } else if (AllIBTransports.has(pimpl_->config_.transport)) {
#if defined(MSCCLPP_USE_IB)
    detail::serialize(data, pimpl_->ibQpInfo_);
    detail::serialize(data, pimpl_->ibNoAtomic_);
#else
    throw Error("IB support is disabled", ErrorCode::InvalidUsage);
#endif
  } else if (pimpl_->config_.transport == Transport::Ethernet) {
    detail::serialize(data, pimpl_->socketAddress_);
  }
  return data;
}

MSCCLPP_API_CPP Endpoint Endpoint::deserialize(const std::vector<char>& data) {
  return Endpoint(std::make_shared<Impl>(data));
}

}  // namespace mscclpp
