// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#include "mrc.hpp"

#if defined(MSCCLPP_USE_MRC)

#include <mrc.h>
#include <unistd.h>

#include <cerrno>
#include <limits>
#include <mscclpp/errors.hpp>
#include <mscclpp/gpu_utils.hpp>

#if defined(MSCCLPP_USE_MLX5DV)
#include "mlx5dv_wrapper.hpp"
#endif

namespace mscclpp {

static_assert(MRC_API_CURRENT_VERSION == MRC_API_VER(1, 0, 0));
static_assert(MRC_API_VER_USED == MRC_API_VER(1, 0, 0));

MrcMr::MrcMr(ibv_pd* pd, void* buff, std::size_t size, [[maybe_unused]] bool isDataDirect) : mr_(nullptr), buff_(buff) {
  if (buff == nullptr || size == 0) {
    throw Error("Native MRC requires a nonnull MR buffer and positive size", ErrorCode::InvalidUsage);
  }
  constexpr int accessFlags = IBV_ACCESS_LOCAL_WRITE | IBV_ACCESS_REMOTE_WRITE;

  long pageSize = sysconf(_SC_PAGESIZE);
  if (pageSize <= 0) {
    throw Error("Unable to query page size for native MRC registration", ErrorCode::SystemError);
  }
  uintptr_t bufferAddress = reinterpret_cast<uintptr_t>(buff);
  std::size_t offset = bufferAddress % pageSize;
  if (size > std::numeric_limits<std::size_t>::max() - offset - (pageSize - 1)) {
    throw Error("Native MRC registration range is too large", ErrorCode::InvalidUsage);
  }
  uintptr_t address = bufferAddress - offset;
  std::size_t rangeSize = ((size + offset + pageSize - 1) / pageSize) * pageSize;

  int gpuId = -1;
#if defined(MSCCLPP_USE_CUDA)
  CUmemorytype memoryType{};
  CUresult result =
      cuPointerGetAttribute(&memoryType, CU_POINTER_ATTRIBUTE_MEMORY_TYPE, reinterpret_cast<CUdeviceptr>(buff));
  if (result != CUDA_ERROR_NOT_INITIALIZED && result != CUDA_ERROR_INVALID_VALUE) {
    MSCCLPP_CUTHROW(result);
    if (memoryType == CU_MEMORYTYPE_DEVICE || memoryType == CU_MEMORYTYPE_UNIFIED) {
      gpuId = detail::gpuIdFromAddress(buff);
    }
  }
#else
  gpuId = detail::gpuIdFromAddress(buff);
#endif
  CudaDeviceGuard deviceGuard(gpuId);

#if defined(MSCCLPP_USE_CUDA)
  if (gpuId >= 0) {
    CUdevice device;
    MSCCLPP_CUTHROW(cuDeviceGet(&device, gpuId));
    int dmaBufSupported = 0;
    MSCCLPP_CUTHROW(cuDeviceGetAttribute(&dmaBufSupported, CU_DEVICE_ATTRIBUTE_DMA_BUF_SUPPORTED, device));
    if (dmaBufSupported) {
      int fd = -1;
      auto closeFd = [](int* descriptor) {
        if (*descriptor >= 0) ::close(*descriptor);
      };
      // Track fd by address so retries also close the replacement descriptor.
      std::unique_ptr<int, decltype(closeFd)> fdGuard(&fd, closeFd);
      auto exportFd = [&](unsigned long long flags) {
        if (fd >= 0) ::close(fd);
        fd = -1;
        return cuMemGetHandleForAddressRange(&fd, address, rangeSize, CU_MEM_RANGE_HANDLE_TYPE_DMA_BUF_FD, flags);
      };

      bool usedPcieMapping = false;
#if CUDA_VERSION >= 12080
      result = exportFd(CU_MEM_RANGE_FLAG_DMA_BUF_MAPPING_TYPE_PCIE);
      usedPcieMapping = result == CUDA_SUCCESS && fd >= 0;
#endif
      if (!usedPcieMapping) {
        result = exportFd(0);
      }
      MSCCLPP_CUTHROW(result);
      if (fd < 0) {
        throw Error("GPU DMA-BUF export returned an invalid descriptor", ErrorCode::SystemError);
      }

      auto registerFd = [&] {
        ibv_mr* mr = nullptr;
#if defined(MSCCLPP_USE_MLX5DV)
        if (isDataDirect) {
          mr = MLX5DV::mlx5dv_reg_dmabuf_mr(pd, offset, size, bufferAddress, fd, accessFlags);
        }
#endif
        if (mr == nullptr) {
          mr = ibv_reg_dmabuf_mr(pd, offset, size, bufferAddress, fd, accessFlags);
        }
        return mr;
      };
      mr_ = registerFd();
      if (mr_ == nullptr && usedPcieMapping) {
        MSCCLPP_CUTHROW(exportFd(0));
        if (fd < 0) {
          throw Error("GPU DMA-BUF export returned an invalid descriptor", ErrorCode::SystemError);
        }
        mr_ = registerFd();
      }
      if (mr_ == nullptr) {
        throw IbError("DMA-BUF registration failed for native MRC", errno);
      }
      return;
    }
    if (isCuMemMapAllocated(buff)) {
      throw Error("Native MRC VMM registration requires DMA-BUF support", ErrorCode::InvalidUsage);
    }
    if (::access("/sys/module/nvidia_peermem", F_OK) != 0) {
      throw Error("nvidia_peermem kernel module is not loaded", ErrorCode::SystemError);
    }
  }
#endif

  mr_ = ibv_reg_mr(pd, reinterpret_cast<void*>(address), rangeSize, accessFlags);
  if (mr_ == nullptr) {
    throw IbError("ibv_reg_mr failed for native MRC", errno);
  }
}

MrcMr::~MrcMr() { ibv_dereg_mr(mr_); }

MrcMrInfo MrcMr::getInfo() const { return {reinterpret_cast<uint64_t>(buff_), mr_->rkey}; }

const void* MrcMr::getBuff() const { return buff_; }

uint32_t MrcMr::getLkey() const { return mr_->lkey; }

MrcQp::MrcQp(ibv_context* context, mrc_context* mrcContext, ibv_pd* pd, int port, int gidIndex, uint64_t evProfileId,
             int maxSendCqSize, int maxSendWr, int maxRecvWr)
    : portNum_(port), gidIndex_(gidIndex), info_{}, sendCq_(nullptr), recvCq_(nullptr), qp_(nullptr) {
  if (maxSendCqSize <= 0 || maxSendWr <= 0 || maxRecvWr < 0) {
    throw Error("Native MRC requires positive send CQ/WR limits and a nonnegative receive WR limit",
                ErrorCode::InvalidUsage);
  }
  if (port <= 0 || port > 255) {
    throw Error("Native MRC requires a port between 1 and 255", ErrorCode::InvalidUsage);
  }

  std::unique_ptr<mrc_cq, decltype(&mrc_destroy_cq)> sendCq(
      mrc_create_cq(mrcContext, maxSendCqSize, nullptr, nullptr, 0), mrc_destroy_cq);
  if (!sendCq) {
    throw IbError("mrc_create_cq failed for send CQ", errno);
  }

  std::unique_ptr<mrc_cq, decltype(&mrc_destroy_cq)> recvCq(nullptr, mrc_destroy_cq);
  if (maxRecvWr > 0) {
    recvCq.reset(mrc_create_cq(mrcContext, maxRecvWr, nullptr, nullptr, 0));
    if (!recvCq) {
      throw IbError("mrc_create_cq failed for receive CQ", errno);
    }
  }

  mrc_qp_init_attr attrs{};
  attrs.send_cq = sendCq.get();
  attrs.recv_cq = recvCq ? recvCq.get() : sendCq.get();
  attrs.cap.max_send_wr = maxSendWr;
  attrs.cap.max_recv_wr = maxRecvWr;
  attrs.cap.max_send_sge = 1;
  attrs.cap.max_recv_sge = 1;
  attrs.pd = pd;
  std::unique_ptr<mrc_qp, decltype(&mrc_destroy_qp)> qp(mrc_create_qp(mrcContext, &attrs), mrc_destroy_qp);
  if (!qp) {
    throw IbError("mrc_create_qp failed", errno);
  }

  ibv_port_attr portAttr{};
  int ret = ibv_query_port(context, portNum_, &portAttr);
  if (ret != 0) {
    throw IbError("ibv_query_port failed for native MRC QP", ret > 0 ? ret : errno);
  }
  info_.lid = portAttr.lid;
  info_.linkLayer = portAttr.link_layer;
  info_.mtu = portAttr.active_mtu;
  info_.isGrh = (portAttr.flags & IBV_QPF_GRH_REQUIRED);

  ret = mrc_get_qpn(qp.get(), &info_.qpn);
  if (ret != 0) {
    throw IbError("mrc_get_qpn failed", ret > 0 ? ret : errno);
  }

  if (portAttr.link_layer != IBV_LINK_LAYER_INFINIBAND || info_.isGrh) {
    if (gidIndex_ < 0 || gidIndex_ >= portAttr.gid_tbl_len) {
      throw Error("invalid native MRC GID index " + std::to_string(gidIndex_) + " for port " +
                      std::to_string(portNum_) + " (GID table length " + std::to_string(portAttr.gid_tbl_len) + ")",
                  ErrorCode::InvalidUsage);
    }
    ibv_gid gid{};
    ret = ibv_query_gid(context, portNum_, gidIndex_, &gid);
    if (ret != 0) {
      throw IbError("ibv_query_gid failed for native MRC QP", ret > 0 ? ret : errno);
    }
    info_.spn = gid.global.subnet_prefix;
    info_.iid = gid.global.interface_id;
  }

  ibv_qp_attr qpAttrs{};
  qpAttrs.qp_state = IBV_QPS_INIT;
  qpAttrs.port_num = portNum_;
  qpAttrs.pkey_index = 0;
  qpAttrs.qp_access_flags = IBV_ACCESS_REMOTE_WRITE;

  mrc_qp_attr mrcAttrs{};
  mrcAttrs.ev_profile_id = evProfileId;
  ret = mrc_modify_qp(qp.get(), &qpAttrs, IBV_QP_STATE | IBV_QP_PORT | IBV_QP_PKEY_INDEX | IBV_QP_ACCESS_FLAGS,
                      &mrcAttrs, MRC_QP_EV_PROFILE_ID);
  if (ret != 0) {
    throw IbError("mrc_modify_qp failed transitioning to INIT", ret > 0 ? ret : errno);
  }

  sendCq_ = sendCq.release();
  recvCq_ = recvCq.release();
  qp_ = qp.release();
}

MrcQp::~MrcQp() {
  mrc_destroy_qp(qp_);
  mrc_destroy_cq(sendCq_);
  if (recvCq_ != nullptr) {
    mrc_destroy_cq(recvCq_);
  }
}

void MrcQp::rtr(const MrcQpInfo& info) {
  ibv_qp_attr qpAttrs{};
  qpAttrs.qp_state = IBV_QPS_RTR;
  qpAttrs.path_mtu = static_cast<ibv_mtu>(info.mtu);
  qpAttrs.dest_qp_num = info.qpn;
  qpAttrs.rq_psn = 0;
  if (info.linkLayer == IBV_LINK_LAYER_ETHERNET || info.isGrh) {
    qpAttrs.ah_attr.is_global = 1;
    qpAttrs.ah_attr.grh.dgid.global.subnet_prefix = info.spn;
    qpAttrs.ah_attr.grh.dgid.global.interface_id = info.iid;
    qpAttrs.ah_attr.grh.sgid_index = gidIndex_;
    qpAttrs.ah_attr.grh.hop_limit = 255;
    // Match the reference program's traffic class on the installed provider.
    qpAttrs.ah_attr.grh.traffic_class = 1;
  }
  qpAttrs.ah_attr.dlid = info.lid;
  qpAttrs.ah_attr.port_num = portNum_;

  mrc_qp_attr mrcAttrs{};
  mrcAttrs.version = MRC_VERSION_1;
  int ret = mrc_modify_qp(qp_, &qpAttrs, IBV_QP_STATE | IBV_QP_AV | IBV_QP_PATH_MTU | IBV_QP_DEST_QPN | IBV_QP_RQ_PSN,
                          &mrcAttrs, MRC_QP_PROTOCOL_VERSION);
  if (ret != 0) {
    throw IbError("mrc_modify_qp failed transitioning to RTR", ret > 0 ? ret : errno);
  }
}

void MrcQp::rts() {
  ibv_qp_attr qpAttrs{};
  qpAttrs.qp_state = IBV_QPS_RTS;
  qpAttrs.sq_psn = 0;
  // The installed provider accepts these verbs retry fields despite the SDK's unsupported-fields table.
  // FIXME: should this be set dynamically based on some environment variable?
  qpAttrs.retry_cnt = 7;
  qpAttrs.rnr_retry = 7;
  mrc_qp_attr mrcAttrs{};
  mrcAttrs.timeout = 14;
  int ret = mrc_modify_qp(qp_, &qpAttrs, IBV_QP_STATE | IBV_QP_SQ_PSN | IBV_QP_RETRY_CNT | IBV_QP_RNR_RETRY, &mrcAttrs,
                          MRC_QP_TIMEOUT);
  if (ret != 0) {
    throw IbError("mrc_modify_qp failed transitioning to RTS", ret > 0 ? ret : errno);
  }
}

MrcCtx::MrcCtx(const std::string& devName)
    : devName_(devName), context_(nullptr), pd_(nullptr), mrcContext_(nullptr), isDataDirect_(false) {
  if (devName.empty()) {
    throw Error("Native MRC requires a device name", ErrorCode::InvalidUsage);
  }

  int numDevices = 0;
  std::unique_ptr<ibv_device*, decltype(&ibv_free_device_list)> devices(ibv_get_device_list(&numDevices),
                                                                        ibv_free_device_list);
  if (!devices) {
    throw IbError("ibv_get_device_list failed", errno);
  }
  std::unique_ptr<ibv_context, decltype(&ibv_close_device)> context(nullptr, ibv_close_device);
  for (int i = 0; i < numDevices; ++i) {
    if (devName == ibv_get_device_name(devices.get()[i])) {
      context.reset(ibv_open_device(devices.get()[i]));
      if (!context) {
        throw IbError("ibv_open_device failed for " + devName, errno);
      }
      break;
    }
  }
  if (!context) {
    throw Error("Native MRC device not found: " + devName, ErrorCode::InvalidUsage);
  }

  // TODO: Use mrc_query_device_ex and its attribute layout when adding newer SDK support.
  mrc_attr capabilities{};
  int supported = 0;
  if (mrc_query_device(context.get(), &capabilities, &supported) != 0) {
    throw IbError("mrc_query_device failed for " + devName, errno);
  }
  if (!supported) {
    throw Error("Device does not support native MRC: " + devName, ErrorCode::InvalidUsage);
  }

#if defined(MSCCLPP_USE_MLX5DV)
  if (MLX5DV::isAvailable() && MLX5DV::mlx5dv_is_supported(context->device)) {
    char sysfsPath[256];
    isDataDirect_ = MLX5DV::mlx5dv_get_data_direct_sysfs_path(context.get(), sysfsPath, sizeof(sysfsPath)) == 0;
  }
#endif

  std::unique_ptr<ibv_pd, decltype(&ibv_dealloc_pd)> pd(ibv_alloc_pd(context.get()), ibv_dealloc_pd);
  if (!pd) {
    throw IbError("ibv_alloc_pd failed for " + devName, errno);
  }

  // TODO: Add runtime compatibility checks with mrc_query_version when the SDK provides it.
  mrc_context_attr attrs{};
  attrs.mrc_api_version_used = MRC_API_VER(1, 0, 0);
  std::unique_ptr<mrc_context, decltype(&mrc_destroy_context)> mrcContext(mrc_create_context(context.get(), &attrs),
                                                                          mrc_destroy_context);
  if (!mrcContext) {
    throw IbError("mrc_create_context failed for " + devName, errno);
  }

  context_ = context.release();
  pd_ = pd.release();
  mrcContext_ = mrcContext.release();
}

MrcCtx::~MrcCtx() {
  mrc_destroy_context(mrcContext_);
  ibv_dealloc_pd(pd_);
  ibv_close_device(context_);
}

std::shared_ptr<MrcQp> MrcCtx::createQp(int port, int gidIndex, uint64_t evProfileId, int maxSendCqSize, int maxSendWr,
                                        int maxRecvWr) {
  return std::shared_ptr<MrcQp>(
      new MrcQp(context_, mrcContext_, pd_, port, gidIndex, evProfileId, maxSendCqSize, maxSendWr, maxRecvWr));
}

std::unique_ptr<const MrcMr> MrcCtx::registerMr(void* buff, std::size_t size) {
  return std::unique_ptr<const MrcMr>(new MrcMr(pd_, buff, size, isDataDirect_));
}

}  // namespace mscclpp

#endif  // defined(MSCCLPP_USE_MRC)
