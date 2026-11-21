// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#include "ib.hpp"

#if defined(ENABLE_NPKIT)
#include <mscclpp/npkit/npkit.hpp>
#endif

#include <arpa/inet.h>
#include <malloc.h>
#include <unistd.h>

#include <atomic>
#include <cstring>
#include <fstream>
#include <mscclpp/core.hpp>
#include <mscclpp/env.hpp>
#include <mscclpp/errors.hpp>
#include <mscclpp/fifo.hpp>
#include <mscclpp/numa.hpp>
#include <mscclpp/utils.hpp>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>

#include "api.h"
#include "atomic.hpp"
#include "connection.hpp"
#include "gdr.hpp"
#include "gpunetio.hpp"
#if defined(USE_IBVERBS)
#include "ibverbs_wrapper.hpp"
#if defined(MSCCLPP_USE_MRC)
#include <mrc.h>
#endif
#if defined(MSCCLPP_USE_MLX5DV)
#include "mlx5dv_wrapper.hpp"
#endif  // defined(MSCCLPP_USE_MLX5DV)
#endif  // defined(USE_IBVERBS)
#include "logger.hpp"
#include "mrc_hints.hpp"
#include "serialization.hpp"

#if !defined(MSCCLPP_USE_ROCM)

// Check if nvidia_peermem kernel module is loaded
[[maybe_unused]] static bool checkNvPeerMemLoaded() {
  std::ifstream file("/proc/modules");
  std::string line;
  while (std::getline(file, line)) {
    if (line.find("nvidia_peermem") != std::string::npos) return true;
  }
  return false;
}

#endif  // !defined(MSCCLPP_USE_ROCM)

namespace mscclpp {

#if defined(USE_IBVERBS)

Ib::Context* Ib::createContext(ibv_context* context) { return context; }
void Ib::destroyContext(Context*) {}
Ib::Cq* Ib::createCq(Context* context, int size) { return Verbs::ibv_create_cq(context, size, nullptr, nullptr, 0); }
void Ib::destroyCq(Cq* cq) { Verbs::ibv_destroy_cq(cq); }
Ib::Hint* Ib::createHint(Context*) { return nullptr; }
int Ib::destroyHint(Hint*) { return 0; }
Ib::Qp* Ib::createQp(Context*, ibv_pd* pd, Cq* sendCq, Cq* recvCq, const ibv_qp_cap& cap) {
  ibv_qp_init_attr attr = {};
  attr.send_cq = sendCq;
  attr.recv_cq = recvCq;
  attr.qp_type = IBV_QPT_RC;
  attr.cap = cap;
  return Verbs::ibv_create_qp(pd, &attr);
}
int Ib::destroyQp(Qp* qp) { return Verbs::ibv_destroy_qp(qp); }
int Ib::getQpn(Qp* qp, uint32_t* qpn) {
  *qpn = qp->qp_num;
  return 0;
}
int Ib::modifyQp(Qp* qp, ibv_qp_attr* attr, int mask, Hint*) { return Verbs::ibv_modify_qp(qp, attr, mask); }
int Ib::postSend(Qp* qp, ibv_send_wr* wr, ibv_send_wr** badWr) { return Verbs::ibv_post_send(qp, wr, badWr); }
int Ib::postRecv(Qp* qp, ibv_recv_wr* wr, ibv_recv_wr** badWr) { return Verbs::ibv_post_recv(qp, wr, badWr); }
int Ib::pollCq(Cq* cq, int count, ibv_wc* wc) { return Verbs::ibv_poll_cq(cq, count, wc); }

#if defined(MSCCLPP_USE_MRC)
Mrc::Context* Mrc::createContext(ibv_context* context) {
  mrc_attr attr = {};
  int supported = 0;
  int ret = ::mrc_query_device(context, &attr, &supported);
  if (ret != 0) {
    THROW(NET, IbError, ret < 0 ? errno : ret, "mrc_query_device failed");
  }
  if (!supported) {
    THROW(NET, Error, ErrorCode::InvalidUsage, "Selected device does not support native MRC");
  }
  mrc_context_attr contextAttr = {};
  contextAttr.mrc_api_version_used = MRC_API_VER_USED;
  return ::mrc_create_context(context, &contextAttr);
}
void Mrc::destroyContext(Context* context) { (void)::mrc_destroy_context(context); }
Mrc::Cq* Mrc::createCq(Context* context, int size) { return ::mrc_create_cq(context, size, nullptr, nullptr, 0); }
void Mrc::destroyCq(Cq* cq) { (void)::mrc_destroy_cq(cq); }
Mrc::Hint* Mrc::createHint(Context* context) {
  const auto config = env();
  if (!config->mrcQpHintEnable) return nullptr;
  // MSCCL++ currently owns one QP per connection. Preserve the plugin's
  // connection-wide MIN_RATE / QP-count semantics without inventing a mesh size.
  constexpr int numQpsPerPeer = 1;
  const auto cc =
      detail::makeMrcCcHint(config->mrcCcInitRate, config->mrcCcMinRate, config->mrcCcMaxRate, numQpsPerPeer);
  static_assert(sizeof(cc) <= MRC_MAX_VENDOR_CFG_SIZE);
  mrc_qp_hint_init_attr attr{};
  attr.attr.num_qps_per_peer = numQpsPerPeer;
  attr.attr.num_send_peers = 1;
  std::memcpy(attr.attr.vendor_cfg, &cc, sizeof(cc));
  INFO(NET, "Native MRC QP hint num_qps_per_peer=1 num_send_peers=1 cc_version=", cc.version,
       " cc_init_rate=", cc.initRate, " cc_min_rate=", cc.minRate, " cc_max_rate=", cc.maxRate);
  auto* hint = ::mrc_create_qp_hint(context, &attr);
  if (!hint) {
    THROW(NET, IbError, errno, "mrc_create_qp_hint failed (errno ", errno, ")");
  }
  return hint;
}
int Mrc::destroyHint(Hint* hint) { return ::mrc_destroy_qp_hint(hint); }
Mrc::Qp* Mrc::createQp(Context* context, ibv_pd* pd, Cq* sendCq, Cq* recvCq, const ibv_qp_cap& cap) {
  mrc_qp_init_attr attr = {};
  attr.send_cq = sendCq;
  attr.recv_cq = recvCq;
  attr.pd = pd;
  attr.cap = cap;
  return ::mrc_create_qp(context, &attr);
}
int Mrc::destroyQp(Qp* qp) { return ::mrc_destroy_qp(qp); }
int Mrc::getQpn(Qp* qp, uint32_t* qpn) { return ::mrc_get_qpn(qp, qpn); }
int Mrc::modifyQp(Qp* qp, ibv_qp_attr* attr, int mask, Hint* hint) {
  mrc_qp_attr mrcAttr = {};
  int mrcMask = 0;
  if (hint && (mask & IBV_QP_STATE) && attr->qp_state == IBV_QPS_INIT) {
    mrcAttr.qp_hint = hint;
    mrcMask |= MRC_QP_HINT;
  }
  if (mask & IBV_QP_TIMEOUT) {
    // MRC uses 1.024us rather than IB's 4.096us base. Preserve the nominal
    // timeout (IB exponent 18 -> MRC exponent 20), not the packet retry policy.
    if (attr->timeout > 22) {
      THROW(NET, Error, ErrorCode::InvalidUsage, "Timeout exceeds the native MRC limit");
    }
    mrcAttr.timeout = attr->timeout + 2;
    mrcMask |= MRC_QP_TIMEOUT;
  }
  // Keep PORT and retry/RNR fields, as in the native plugin. Despite the
  // header's unsupported-mask list, DOCA 3.2 uses these fields: omitting PORT
  // leaves port 0; omitting retry/RNR leaves zero retries (verified by query).
  // Timeout uses the native mask; read/atomic and alternate-path masks do not.
  mask &=
      ~(IBV_QP_TIMEOUT | IBV_QP_MAX_QP_RD_ATOMIC | IBV_QP_MAX_DEST_RD_ATOMIC | IBV_QP_ALT_PATH | IBV_QP_PATH_MIG_STATE);
  return ::mrc_modify_qp(qp, attr, mask, &mrcAttr, mrcMask);
}
int Mrc::postSend(Qp* qp, ibv_send_wr* wr, ibv_send_wr** badWr) { return ::mrc_post_send(qp, wr, badWr); }
int Mrc::postRecv(Qp* qp, ibv_recv_wr* wr, ibv_recv_wr** badWr) { return ::mrc_post_recv(qp, wr, badWr); }
int Mrc::pollCq(Cq* cq, int count, ibv_wc* wc) { return ::mrc_poll_cq(cq, count, wc); }
#else
Mrc::Context* Mrc::createContext(ibv_context*) {
  THROW(NET, Error, ErrorCode::InvalidUsage, "Native MRC support is not enabled");
}
void Mrc::destroyContext(Context*) {}
Mrc::Cq* Mrc::createCq(Context*, int) {
  THROW(NET, Error, ErrorCode::InvalidUsage, "Native MRC support is not enabled");
}
void Mrc::destroyCq(Cq*) {}
Mrc::Hint* Mrc::createHint(Context*) {
  THROW(NET, Error, ErrorCode::InvalidUsage, "Native MRC support is not enabled");
}
int Mrc::destroyHint(Hint*) { return 0; }
Mrc::Qp* Mrc::createQp(Context*, ibv_pd*, Cq*, Cq*, const ibv_qp_cap&) {
  THROW(NET, Error, ErrorCode::InvalidUsage, "Native MRC support is not enabled");
}
int Mrc::destroyQp(Qp*) { return 0; }
int Mrc::getQpn(Qp*, uint32_t*) { THROW(NET, Error, ErrorCode::InvalidUsage, "Native MRC support is not enabled"); }
int Mrc::modifyQp(Qp*, ibv_qp_attr*, int, Hint*) {
  THROW(NET, Error, ErrorCode::InvalidUsage, "Native MRC support is not enabled");
}
int Mrc::postSend(Qp*, ibv_send_wr*, ibv_send_wr**) {
  THROW(NET, Error, ErrorCode::InvalidUsage, "Native MRC support is not enabled");
}
int Mrc::postRecv(Qp*, ibv_recv_wr*, ibv_recv_wr**) {
  THROW(NET, Error, ErrorCode::InvalidUsage, "Native MRC support is not enabled");
}
int Mrc::pollCq(Cq*, int, ibv_wc*) { THROW(NET, Error, ErrorCode::InvalidUsage, "Native MRC support is not enabled"); }
#endif

template <typename Verbs>
static inline bool isDmabufSupportedByGpu(int gpuId) {
  static std::unordered_map<int, bool> cache;
  if (gpuId < 0 || !Verbs::isDmabufSupported()) {
    return false;
  }
  if (cache.find(gpuId) != cache.end()) {
    return cache[gpuId];
  }
  int dmaBufSupported = 0;
#if !defined(MSCCLPP_USE_ROCM)
  CUdevice dev;
  MSCCLPP_CUTHROW(cuDeviceGet(&dev, gpuId));
  MSCCLPP_CUTHROW(cuDeviceGetAttribute(&dmaBufSupported, CU_DEVICE_ATTRIBUTE_DMA_BUF_SUPPORTED, dev));
#endif  // !defined(MSCCLPP_USE_ROCM)
  bool ret = dmaBufSupported != 0;
  if (!ret) {
    DEBUG(NET, "GPU ", gpuId, " does not support DMABUF");
  }
  cache[gpuId] = ret;
  return ret;
}

template <typename Verbs>
static ibv_mr* registerVerbsMr(ibv_pd* pd, void* buff, std::size_t size, [[maybe_unused]] bool isDataDirect,
                               std::size_t& registeredSize) {
  ibv_mr* mr = nullptr;
  if (size == 0) {
    THROW(NET, Error, ErrorCode::InvalidUsage, "invalid MR size: 0");
  }
  static __thread uintptr_t pageSize = 0;
  if (pageSize == 0) {
    pageSize = sysconf(_SC_PAGESIZE);
  }
  uintptr_t buffIntPtr = reinterpret_cast<uintptr_t>(buff);
  uintptr_t addr = buffIntPtr & -pageSize;
  std::size_t pages = (size + (buffIntPtr - addr) + pageSize - 1) / pageSize;

  int gpuId = detail::gpuIdFromAddress(buff);
  bool isGpuBuff = (gpuId != -1);
  if (isGpuBuff && isDmabufSupportedByGpu<Verbs>(gpuId)) {
#if !defined(MSCCLPP_USE_ROCM)
    int fd = -1;
    auto closeFd = [](int* fd) {
      if (*fd >= 0) ::close(*fd);
    };
    std::unique_ptr<int, decltype(closeFd)> fdGuard(&fd, closeFd);
    size_t rangeSize = pages * pageSize;

    // Obtain a DMA-BUF file descriptor for the GPU memory range. On platforms with a CPU-GPU
    // bridge that reorders posted writes (e.g., Grace/GB200 NVLink-C2C), the PCIe mapping flag
    // routes DMA through the Data Direct engine for correct ordering and higher throughput.
    // Fall back to the default (non-PCIe) mapping if the flag is unsupported.
#if (CUDA_VERSION >= 12080)
    CUresult cuRes = cuMemGetHandleForAddressRange(&fd, addr, rangeSize, CU_MEM_RANGE_HANDLE_TYPE_DMA_BUF_FD,
                                                   CU_MEM_RANGE_FLAG_DMA_BUF_MAPPING_TYPE_PCIE);
    if (cuRes != CUDA_SUCCESS || fd < 0) {
      if (fd >= 0) ::close(fd);
      fd = -1;
    }
    bool usedPcieFlag = (fd >= 0);
#endif  // CUDA_VERSION >= 12080
    if (fd < 0) {
      MSCCLPP_CUTHROW(cuMemGetHandleForAddressRange(&fd, addr, rangeSize, CU_MEM_RANGE_HANDLE_TYPE_DMA_BUF_FD, 0));
    }

    // Register the DMA-BUF memory region. When Data Direct is available, use the mlx5dv API
    // which enables hardware-level Data Direct routing for the MR. Otherwise use standard verbs.
    size_t offsetInDmaBuf = buffIntPtr % pageSize;
    int accessFlags = IBV_ACCESS_LOCAL_WRITE | IBV_ACCESS_REMOTE_WRITE | IBV_ACCESS_REMOTE_READ |
                      IBV_ACCESS_RELAXED_ORDERING | IBV_ACCESS_REMOTE_ATOMIC;

#if defined(MSCCLPP_USE_MLX5DV)
    if (isDataDirect) {
      mr = MLX5DV::mlx5dv_reg_dmabuf_mr(pd, offsetInDmaBuf, size, buffIntPtr, fd, accessFlags);
    }
#endif
    if (mr == nullptr) {
      mr = Verbs::ibv_reg_dmabuf_mr(pd, offsetInDmaBuf, size, buffIntPtr, fd, accessFlags);
    }

    // If MR registration failed with a PCIe-mapped fd, retry with the default mapping.
#if (CUDA_VERSION >= 12080)
    if (mr == nullptr && usedPcieFlag) {
      ::close(fd);
      fd = -1;
      MSCCLPP_CUTHROW(cuMemGetHandleForAddressRange(&fd, addr, rangeSize, CU_MEM_RANGE_HANDLE_TYPE_DMA_BUF_FD, 0));
      mr = Verbs::ibv_reg_dmabuf_mr(pd, offsetInDmaBuf, size, buffIntPtr, fd, accessFlags);
    }
#endif  // CUDA_VERSION >= 12080

    if (mr == nullptr) {
      THROW(NET, IbError, errno, "ibv_reg_dmabuf_mr failed (errno ", errno, ")");
    }
#else   // defined(MSCCLPP_USE_ROCM)
    THROW(NET, Error, ErrorCode::InvalidUsage, "We don't support DMABUF on HIP platforms yet");
#endif  // defined(MSCCLPP_USE_ROCM)
  } else {
#if !defined(MSCCLPP_USE_ROCM)
    if (isGpuBuff) {
      if (isCuMemMapAllocated(buff)) {
        THROW(NET, Error, ErrorCode::InvalidUsage, "DMABUF is required but is not supported in this platform.");
      }
      // Need nvidia-peermem when DMABUF is not supported
      if (!checkNvPeerMemLoaded()) {
        THROW(NET, Error, ErrorCode::SystemError, "nvidia_peermem kernel module is not loaded");
      }
    }
#endif  // !defined(MSCCLPP_USE_ROCM)
    mr = Verbs::ibv_reg_mr(pd, reinterpret_cast<void*>(addr), pages * pageSize,
                           IBV_ACCESS_LOCAL_WRITE | IBV_ACCESS_REMOTE_WRITE | IBV_ACCESS_REMOTE_READ |
                               IBV_ACCESS_RELAXED_ORDERING | IBV_ACCESS_REMOTE_ATOMIC);
    if (mr == nullptr) {
      THROW(NET, IbError, errno, "ibv_reg_mr failed (errno ", errno, ")");
    }
  }

  registeredSize = pages * pageSize;
  return mr;
}

IbMr::IbMr(ibv_pd* pd, void* buff, std::size_t size, bool isDataDirect, bool native)
    : mr_(nullptr), buff_(buff), size_(0), native_(native) {
  mr_ = native_ ? registerVerbsMr<NativeIBVerbs>(pd, buff, size, isDataDirect, size_)
                : registerVerbsMr<IBVerbs>(pd, buff, size, isDataDirect, size_);
}

IbMr::~IbMr() {
  if (native_)
    NativeIBVerbs::ibv_dereg_mr(mr_);
  else
    IBVerbs::ibv_dereg_mr(mr_);
}

IbMrInfo IbMr::getInfo() const {
  IbMrInfo info;
  info.addr = reinterpret_cast<uint64_t>(buff_);
  info.rkey = mr_->rkey;
  return info;
}

const void* IbMr::getBuff() const { return buff_; }

uint32_t IbMr::getLkey() const { return mr_->lkey; }

template <typename Backend>
VerbsQp<Backend>::VerbsQp(typename Backend::Context* ctx, ibv_context* verbsCtx, ibv_pd* pd, int portNum, int gidIndex,
                          int maxSendCqSize, int maxSendCqPollNum, int maxSendWr, int maxRecvWr, int maxWrPerSend,
                          bool noAtomic)
    : portNum_(portNum),
      gidIndex_(gidIndex),
      info_(),
      qp_(nullptr),
      hint_(nullptr),
      sendCq_(nullptr),
      recvCq_(nullptr),
      sendWcs_(),
      recvWcs_(),
      sendWrs_(),
      sendSges_(),
      recvWrs_(),
      recvSges_(),
      numStagedSend_(0),
      numStagedRecv_(0),
      numPostedSignaledSend_(0),
      numStagedSignaledSend_(0),
      maxSendCqSize_(maxSendCqSize),
      maxSendCqPollNum_(maxSendCqPollNum),
      maxSendWr_(maxSendWr),
      maxWrPerSend_(maxWrPerSend),
      maxRecvWr_(maxRecvWr),
      noAtomic_(noAtomic) {
  if (maxSendCqSize <= 0 || maxSendCqPollNum <= 0 || maxSendWr <= 0 || maxWrPerSend <= 0 || maxRecvWr < 0 ||
      (noAtomic && maxRecvWr == 0)) {
    THROW(NET, Error, ErrorCode::InvalidUsage, "Invalid RDMA queue sizes");
  }
  try {
    hint_ = Backend::createHint(ctx);
    sendCq_ = Backend::createCq(ctx, maxSendCqSize);
    if (sendCq_ == nullptr) {
      THROW(NET, IbError, errno, Backend::name, " create CQ failed (errno ", errno, ")");
    }

    // Only create recv CQ if maxRecvWr > 0
    if (maxRecvWr > 0) {
      recvCq_ = Backend::createCq(ctx, maxRecvWr);
      if (recvCq_ == nullptr) {
        THROW(NET, IbError, errno, Backend::name, " create CQ failed (errno ", errno, ")");
      }
    }

    ibv_qp_cap cap = {};
    // Use separate recv CQ if created, otherwise use the send CQ
    cap.max_send_wr = maxSendWr;
    cap.max_recv_wr = maxRecvWr;
    cap.max_send_sge = 1;
    cap.max_recv_sge = 1;
    cap.max_inline_data = 0;

    qp_ = Backend::createQp(ctx, pd, sendCq_, recvCq_ ? recvCq_ : sendCq_, cap);
    if (qp_ == nullptr) {
      THROW(NET, IbError, errno, Backend::name, " create QP failed (errno ", errno, ")");
    }

    struct ibv_port_attr portAttr;
    if (Verbs::ibv_query_port(verbsCtx, portNum_, &portAttr) != 0) {
      THROW(NET, IbError, errno, "ibv_query_port failed (errno ", errno, ")");
    }
    info_.lid = portAttr.lid;
    info_.linkLayer = portAttr.link_layer;
    if (Backend::getQpn(qp_, &info_.qpn) != 0) {
      THROW(NET, IbError, errno, Backend::name, " get QPN failed (errno ", errno, ")");
    }
    info_.mtu = portAttr.active_mtu;
    info_.isGrh = (portAttr.flags & IBV_QPF_GRH_REQUIRED);

    if (portAttr.link_layer != IBV_LINK_LAYER_INFINIBAND || info_.isGrh) {
      if (gidIndex_ < 0 || gidIndex_ >= portAttr.gid_tbl_len) {
        THROW(NET, Error, ErrorCode::InvalidUsage, "invalid GID index ", gidIndex_, " for port ", portNum_,
              " (max index is ", portAttr.gid_tbl_len - 1, ")");
      }

      union ibv_gid gid = {};
      if (Verbs::ibv_query_gid(verbsCtx, portNum_, gidIndex_, &gid) != 0) {
        THROW(NET, IbError, errno, "ibv_query_gid failed for port ", portNum_, " index ", gidIndex_, " (errno ", errno,
              ")");
      }
      info_.spn = gid.global.subnet_prefix;
      info_.iid = gid.global.interface_id;
    }

    struct ibv_qp_attr qpAttr = {};
    qpAttr.qp_state = IBV_QPS_INIT;
    qpAttr.pkey_index = 0;
    qpAttr.port_num = portNum_;
    qpAttr.qp_access_flags = noAtomic_ ? IBV_ACCESS_REMOTE_WRITE
                                       : (IBV_ACCESS_REMOTE_WRITE | IBV_ACCESS_REMOTE_READ | IBV_ACCESS_REMOTE_ATOMIC);
    int ret =
        Backend::modifyQp(qp_, &qpAttr, IBV_QP_STATE | IBV_QP_PKEY_INDEX | IBV_QP_PORT | IBV_QP_ACCESS_FLAGS, hint_);
    if (ret != 0) {
      THROW(NET, IbError, ret < 0 ? errno : ret, Backend::name, " modify QP failed");
    }
    sendWrs_ = std::make_shared<std::vector<ibv_send_wr>>(maxWrPerSend_);
    sendSges_ = std::make_shared<std::vector<ibv_sge>>(maxWrPerSend_);
    sendWcs_ = std::make_shared<std::vector<ibv_wc>>(maxSendCqPollNum_);
    recvWcs_ = std::make_shared<std::vector<ibv_wc>>(maxRecvWr_);
    if (maxRecvWr_ > 0) {
      recvWrs_ = std::make_shared<std::vector<ibv_recv_wr>>(maxRecvWr_);
      recvSges_ = std::make_shared<std::vector<ibv_sge>>(maxRecvWr_);
    }
  } catch (...) {
    reset();
    throw;
  }
}

template <typename Backend>
VerbsQp<Backend>::~VerbsQp() {
  reset();
}

template <typename Backend>
void VerbsQp<Backend>::reset() {
  if (!detail::destroyQpAndHint<Backend>(qp_, hint_)) {
    WARN(NET, Backend::name, " QP/hint cleanup failed; retaining dependent resources");
    return;
  }
  if (sendCq_) Backend::destroyCq(sendCq_);
  if (recvCq_) Backend::destroyCq(recvCq_);
  qp_ = nullptr;
  sendCq_ = recvCq_ = nullptr;
}

template <typename Backend>
void VerbsQp<Backend>::rtr(const IbQpInfo& info) {
  struct ibv_qp_attr qp_attr = {};
  qp_attr.qp_state = IBV_QPS_RTR;
  qp_attr.path_mtu = static_cast<ibv_mtu>(info.mtu);
  qp_attr.dest_qp_num = info.qpn;
  qp_attr.rq_psn = 0;
  qp_attr.max_dest_rd_atomic = noAtomic_ ? 0 : 1;
  qp_attr.min_rnr_timer = 0x12;
  if (info.linkLayer == IBV_LINK_LAYER_ETHERNET || info.isGrh) {
    qp_attr.ah_attr.is_global = 1;
    qp_attr.ah_attr.grh.dgid.global.subnet_prefix = info.spn;
    qp_attr.ah_attr.grh.dgid.global.interface_id = info.iid;
    qp_attr.ah_attr.grh.flow_label = 0;
    qp_attr.ah_attr.grh.sgid_index = gidIndex_;
    qp_attr.ah_attr.grh.hop_limit = 255;
    qp_attr.ah_attr.grh.traffic_class = 0;
  } else {
    qp_attr.ah_attr.is_global = 0;
  }
  qp_attr.ah_attr.dlid = info.lid;
  qp_attr.ah_attr.sl = 0;
  qp_attr.ah_attr.src_path_bits = 0;
  qp_attr.ah_attr.port_num = portNum_;
  int ret = Backend::modifyQp(qp_, &qp_attr,
                              IBV_QP_STATE | IBV_QP_AV | IBV_QP_PATH_MTU | IBV_QP_DEST_QPN | IBV_QP_RQ_PSN |
                                  IBV_QP_MAX_DEST_RD_ATOMIC | IBV_QP_MIN_RNR_TIMER);
  if (ret != 0) {
    THROW(NET, IbError, ret < 0 ? errno : ret, Backend::name, " modify QP failed");
  }
}

template <typename Backend>
void VerbsQp<Backend>::rts() {
  struct ibv_qp_attr qp_attr = {};
  qp_attr.qp_state = IBV_QPS_RTS;
  qp_attr.timeout = 18;
  qp_attr.retry_cnt = 7;
  qp_attr.rnr_retry = 7;
  qp_attr.sq_psn = 0;
  qp_attr.max_rd_atomic = noAtomic_ ? 0 : 1;
  int ret = Backend::modifyQp(
      qp_, &qp_attr,
      IBV_QP_STATE | IBV_QP_TIMEOUT | IBV_QP_RETRY_CNT | IBV_QP_RNR_RETRY | IBV_QP_SQ_PSN | IBV_QP_MAX_QP_RD_ATOMIC);
  if (ret != 0) {
    THROW(NET, IbError, ret < 0 ? errno : ret, Backend::name, " modify QP failed");
  }
}

template <typename Backend>
typename VerbsQp<Backend>::SendWrInfo VerbsQp<Backend>::getNewSendWrInfo() {
  if (numStagedSend_ >= maxWrPerSend_) {
    THROW(NET, Error, ErrorCode::InvalidUsage, "too many staged work requests. limit is ", maxWrPerSend_);
  }
  ibv_send_wr* wr_ = &sendWrs_->data()[numStagedSend_];
  ibv_sge* sge_ = &sendSges_->data()[numStagedSend_];
  wr_->sg_list = sge_;
  wr_->num_sge = 1;
  wr_->next = nullptr;
  if (numStagedSend_ > 0) {
    (*sendWrs_)[numStagedSend_ - 1].next = wr_;
  }
  numStagedSend_++;
  return SendWrInfo{wr_, sge_};
}

template <typename Backend>
void VerbsQp<Backend>::stageSendWrite(const IbMr* mr, const IbMrInfo& info, uint32_t size, uint64_t wrId,
                                      uint64_t srcOffset, uint64_t dstOffset, bool signaled) {
  auto wrInfo = this->getNewSendWrInfo();
  wrInfo.wr->wr_id = wrId;
  wrInfo.wr->opcode = IBV_WR_RDMA_WRITE;
  wrInfo.wr->send_flags = signaled ? IBV_SEND_SIGNALED : 0;
  wrInfo.wr->wr.rdma.remote_addr = (uint64_t)(info.addr) + dstOffset;
  wrInfo.wr->wr.rdma.rkey = info.rkey;
  wrInfo.sge->addr = (uint64_t)(mr->getBuff()) + srcOffset;
  wrInfo.sge->length = size;
  wrInfo.sge->lkey = mr->getLkey();
  if (signaled) numStagedSignaledSend_++;
}

template <typename Backend>
void VerbsQp<Backend>::stageSendAtomicAdd(const IbMr* mr, const IbMrInfo& info, uint64_t wrId, uint64_t dstOffset,
                                          uint64_t addVal, bool signaled) {
  if constexpr (Backend::nativeMrc) {
    THROW(NET, Error, ErrorCode::InvalidUsage, "Native MRC does not support network atomics");
  }
  auto wrInfo = this->getNewSendWrInfo();
  wrInfo.wr->wr_id = wrId;
  wrInfo.wr->opcode = IBV_WR_ATOMIC_FETCH_AND_ADD;
  wrInfo.wr->send_flags = signaled ? IBV_SEND_SIGNALED : 0;
  wrInfo.wr->wr.atomic.remote_addr = (uint64_t)(info.addr) + dstOffset;
  wrInfo.wr->wr.atomic.rkey = info.rkey;
  wrInfo.wr->wr.atomic.compare_add = addVal;
  wrInfo.sge->addr = (uint64_t)(mr->getBuff());
  wrInfo.sge->length = sizeof(uint64_t);  // atomic op is always on uint64_t
  wrInfo.sge->lkey = mr->getLkey();
  if (signaled) numStagedSignaledSend_++;
}

template <typename Backend>
void VerbsQp<Backend>::stageSendWriteWithImm(const IbMr* mr, const IbMrInfo& info, uint32_t size, uint64_t wrId,
                                             uint64_t srcOffset, uint64_t dstOffset, bool signaled,
                                             unsigned int immData) {
  auto wrInfo = this->getNewSendWrInfo();
  wrInfo.wr->wr_id = wrId;
  wrInfo.wr->opcode = IBV_WR_RDMA_WRITE_WITH_IMM;
  wrInfo.wr->send_flags = signaled ? IBV_SEND_SIGNALED : 0;
  wrInfo.wr->wr.rdma.remote_addr = (uint64_t)(info.addr) + dstOffset;
  wrInfo.wr->wr.rdma.rkey = info.rkey;
  wrInfo.wr->imm_data = htonl(immData);
  if (mr != nullptr) {
    wrInfo.sge->addr = (uint64_t)(mr->getBuff()) + srcOffset;
    wrInfo.sge->length = size;
    wrInfo.sge->lkey = mr->getLkey();
  } else {
    // 0-byte write-with-imm: no source buffer needed
    wrInfo.sge->addr = 0;
    wrInfo.sge->length = 0;
    wrInfo.sge->lkey = 0;
  }
  if (signaled) numStagedSignaledSend_++;
}

template <typename Backend>
void VerbsQp<Backend>::postSend() {
  if (numStagedSend_ == 0) {
    return;
  }
  struct ibv_send_wr* bad_wr;
  int err = Backend::postSend(qp_, sendWrs_->data(), &bad_wr);
  if (err != 0) {
    THROW(NET, IbError, err < 0 ? errno : err, Backend::name, " post send failed");
  }
  numStagedSend_ = 0;
  numPostedSignaledSend_ += numStagedSignaledSend_;
  numStagedSignaledSend_ = 0;
  if (numPostedSignaledSend_ + 4 > maxSendCqSize_) {
    INFO(NET, Backend::name, ": CQ is almost full (", numPostedSignaledSend_, " / ", maxSendCqSize_,
         "). The connection needs to be flushed to prevent timeout errors.");
  }
}

template <typename Backend>
typename VerbsQp<Backend>::RecvWrInfo VerbsQp<Backend>::getNewRecvWrInfo() {
  if (numStagedRecv_ >= maxRecvWr_) {
    THROW(NET, Error, ErrorCode::InvalidUsage, "too many outstanding recv work requests. limit is ", maxRecvWr_);
  }
  ibv_recv_wr* wr = &recvWrs_->data()[numStagedRecv_];
  ibv_sge* sge = &recvSges_->data()[numStagedRecv_];
  wr->next = nullptr;
  if (numStagedRecv_ > 0) {
    (*recvWrs_)[numStagedRecv_ - 1].next = wr;
  }
  numStagedRecv_++;
  return RecvWrInfo{wr, sge};
}

template <typename Backend>
void VerbsQp<Backend>::stageRecv(uint64_t wrId) {
  auto wrInfo = this->getNewRecvWrInfo();
  // For RDMA write-with-imm, data goes to remote_addr specified by sender.
  // We only need the recv WR to get the completion notification with imm_data.
  wrInfo.wr->wr_id = wrId;
  wrInfo.wr->sg_list = nullptr;
  wrInfo.wr->num_sge = 0;
}

template <typename Backend>
void VerbsQp<Backend>::stageRecv(const IbMr* mr, uint64_t wrId, uint32_t size, uint64_t offset) {
  auto wrInfo = this->getNewRecvWrInfo();
  wrInfo.wr->wr_id = wrId;
  wrInfo.sge->addr = reinterpret_cast<uint64_t>(mr->getBuff()) + offset;
  wrInfo.sge->length = size;
  wrInfo.sge->lkey = mr->getLkey();
  wrInfo.wr->sg_list = wrInfo.sge;
  wrInfo.wr->num_sge = 1;
}

template <typename Backend>
void VerbsQp<Backend>::postRecv() {
  if (numStagedRecv_ == 0) return;
  struct ibv_recv_wr* bad_wr;
  int err = Backend::postRecv(qp_, recvWrs_->data(), &bad_wr);
  if (err != 0) {
    THROW(NET, IbError, err < 0 ? errno : err, Backend::name, " post recv failed");
  }
  numStagedRecv_ = 0;
}

template <typename Backend>
int VerbsQp<Backend>::pollSendCq() {
  int wcNum = Backend::pollCq(sendCq_, maxSendCqPollNum_, sendWcs_->data());
  if (wcNum > 0) {
    numPostedSignaledSend_ -= wcNum;
  }
  return wcNum;
}

template <typename Backend>
int VerbsQp<Backend>::pollRecvCq() {
  int wcNum = Backend::pollCq(recvCq_, maxRecvWr_, recvWcs_->data());
  return wcNum;
}

template <typename Backend>
int VerbsQp<Backend>::getSendWcStatus(int idx) const {
  return (*sendWcs_)[idx].status;
}

template <typename Backend>
std::string VerbsQp<Backend>::getSendWcStatusString(int idx) const {
  return Verbs::ibv_wc_status_str((*sendWcs_)[idx].status);
}

template <typename Backend>
int VerbsQp<Backend>::getNumSendCqItems() const {
  return numPostedSignaledSend_;
}

template <typename Backend>
int VerbsQp<Backend>::getRecvWcStatus(int idx) const {
  return (*recvWcs_)[idx].status;
}

template <typename Backend>
std::string VerbsQp<Backend>::getRecvWcStatusString(int idx) const {
  return Verbs::ibv_wc_status_str((*recvWcs_)[idx].status);
}

template <typename Backend>
unsigned int VerbsQp<Backend>::getRecvWcImmData(int idx) const {
  return ntohl((*recvWcs_)[idx].imm_data);
}

template <typename Backend>
VerbsCtx<Backend>::VerbsCtx(const std::string& devName)
    : devName_(devName),
      verbsCtx_(nullptr),
      ctx_(nullptr),
      pd_(nullptr),
      supportsRdmaAtomics_(false),
      isMlx5_(false),
      isDataDirect_(false),
      isVF_(false) {
  try {
    int num = 0;
    struct ibv_device** devices = Verbs::ibv_get_device_list(&num);
    auto freeDevices = [](ibv_device** devices) { Verbs::ibv_free_device_list(devices); };
    std::unique_ptr<ibv_device*, decltype(freeDevices)> deviceList(devices, freeDevices);
    if (!devices) {
      THROW(NET, IbError, errno, "ibv_get_device_list failed");
    }
    for (int i = 0; i < num; ++i) {
      if (std::string(devices[i]->name) == devName_) {
        verbsCtx_ = Verbs::ibv_open_device(devices[i]);

        // Detect if this IB device is a Virtual Function (VF).
        // VFs have a 'physfn' sysfs symlink pointing to their parent PF; PFs do not.
        {
          std::string physfnPath = "/sys/class/infiniband/" + devName_ + "/device/physfn";
          isVF_ = (access(physfnPath.c_str(), F_OK) == 0);
          if (isVF_) {
            INFO(NET, Backend::name, " device ", devName_, " is a Virtual Function (Data Direct ordering available)");
          }
        }

#if defined(MSCCLPP_USE_MLX5DV)
        if (MLX5DV::isAvailable()) {
          isMlx5_ = MLX5DV::mlx5dv_is_supported(devices[i]);
          if (isMlx5_) {
            INFO(NET, Backend::name, " device ", devName_, " supports mlx5 Direct Verbs");
          }
        }
#endif  // defined(MSCCLPP_USE_MLX5DV)
        break;
      }
    }
    deviceList.reset();
    if (verbsCtx_ == nullptr) {
      THROW(NET, IbError, errno, "ibv_open_device failed (errno ", errno, ", device name ", devName_, ")");
    }
    ctx_ = Backend::createContext(verbsCtx_);
    if (ctx_ == nullptr) {
      THROW(NET, IbError, errno, Backend::name, " create context failed (device ", devName_, ")");
    }
    pd_ = Verbs::ibv_alloc_pd(verbsCtx_);
    if (pd_ == nullptr) {
      THROW(NET, IbError, errno, "ibv_alloc_pd failed (errno ", errno, ")");
    }

    // Detect Data Direct support via mlx5dv_get_data_direct_sysfs_path
#if defined(MSCCLPP_USE_MLX5DV)
    if (isMlx5_ && MLX5DV::isAvailable()) {
      char sysfsPath[256];
      int ret = MLX5DV::mlx5dv_get_data_direct_sysfs_path(verbsCtx_, sysfsPath, sizeof(sysfsPath));
      if (ret == 0) {
        isDataDirect_ = true;
        INFO(NET, Backend::name, " device ", devName_, " supports Data Direct (sysfs: ", sysfsPath, ")");
      } else {
        INFO(NET, Backend::name, " device ", devName_, " does not support Data Direct");
      }
    }
#endif  // defined(MSCCLPP_USE_MLX5DV)

    // Query and cache RDMA atomics capability
    if constexpr (!Backend::nativeMrc) {
      struct ibv_device_attr attr = {};
      if (Verbs::ibv_query_device(verbsCtx_, &attr) == 0) {
        supportsRdmaAtomics_ = (attr.atomic_cap == IBV_ATOMIC_HCA || attr.atomic_cap == IBV_ATOMIC_GLOB);
      }
    } else {
      INFO(NET, "Native MRC context on ", devName_, " using mrc_* API (no verbs shim or network atomics)");
    }
  } catch (...) {
    reset();
    throw;
  }
}

template <typename Backend>
VerbsCtx<Backend>::~VerbsCtx() {
  reset();
}

template <typename Backend>
void VerbsCtx<Backend>::reset() {
  // Opened on pd_.
  gpuNetIoDevice_.reset();
  if (ctx_) {
    Backend::destroyContext(ctx_);
    ctx_ = nullptr;
  }
  if (pd_ != nullptr) {
    Verbs::ibv_dealloc_pd(pd_);
    pd_ = nullptr;
  }
  if (verbsCtx_ != nullptr) {
    Verbs::ibv_close_device(verbsCtx_);
    verbsCtx_ = nullptr;
  }
}

template <typename Backend>
bool VerbsCtx<Backend>::isPortUsable(int port, int gidIndex) const {
  struct ibv_port_attr portAttr = {};
  if (Verbs::ibv_query_port(verbsCtx_, port, &portAttr) != 0) {
    THROW(NET, IbError, errno, "ibv_query_port failed (errno ", errno, ", port ", port, ")");
  }

  // Check if port is active and has a supported link layer
  if (portAttr.state != IBV_PORT_ACTIVE) {
    return false;
  }
  if (portAttr.link_layer != IBV_LINK_LAYER_ETHERNET && portAttr.link_layer != IBV_LINK_LAYER_INFINIBAND) {
    return false;
  }

  if (gidIndex >= 0) {
    // For Ethernet/RoCE or InfiniBand with GRH, check if GID table has entries
    if (portAttr.link_layer == IBV_LINK_LAYER_ETHERNET || (portAttr.flags & IBV_QPF_GRH_REQUIRED)) {
      if (gidIndex >= portAttr.gid_tbl_len) {
        return false;
      }
      union ibv_gid gid = {};
      if (Verbs::ibv_query_gid(verbsCtx_, port, gidIndex, &gid) != 0) {
        return false;
      }
    }
  }

  return true;
}

template <typename Backend>
int VerbsCtx<Backend>::getAnyUsablePort(int gidIndex) const {
  struct ibv_device_attr devAttr;
  if (Verbs::ibv_query_device(verbsCtx_, &devAttr) != 0) {
    THROW(NET, IbError, errno, "ibv_query_device failed (errno ", errno, ")");
  }
  for (uint8_t port = 1; port <= devAttr.phys_port_cnt; ++port) {
    if (this->isPortUsable(port, gidIndex)) {
      return port;
    }
  }
  return -1;
}

template <typename Backend>
std::shared_ptr<VerbsQp<Backend>> VerbsCtx<Backend>::createQp(int port, int gidIndex, int maxSendCqSize,
                                                              int maxSendCqPollNum, int maxSendWr, int maxRecvWr,
                                                              int maxWrPerSend, bool noAtomic) {
  if (port == -1) {
    port = this->getAnyUsablePort(gidIndex);
    if (port == -1) {
      THROW(NET, Error, ErrorCode::InvalidUsage, "No usable port found (device: ", devName_, ")");
    }
  } else if (!this->isPortUsable(port, gidIndex)) {
    THROW(NET, Error, ErrorCode::InvalidUsage, "invalid IB port: ", port);
  }
  return std::shared_ptr<VerbsQp<Backend>>(new VerbsQp<Backend>(ctx_, verbsCtx_, pd_, port, gidIndex, maxSendCqSize,
                                                                maxSendCqPollNum, maxSendWr, maxRecvWr, maxWrPerSend,
                                                                noAtomic || Backend::nativeMrc));
}

template <typename Backend>
IbQpInfo VerbsCtx<Backend>::getPortInfo(int& port, int gidIndex) const {
  if (port == -1) {
    port = this->getAnyUsablePort(gidIndex);
    if (port == -1) {
      THROW(NET, Error, ErrorCode::InvalidUsage, "No usable port found (device: ", devName_, ")");
    }
  } else if (!this->isPortUsable(port, gidIndex)) {
    THROW(NET, Error, ErrorCode::InvalidUsage, "invalid IB port: ", port);
  }
  struct ibv_port_attr portAttr;
  if (Verbs::ibv_query_port(verbsCtx_, port, &portAttr) != 0) {
    THROW(NET, IbError, errno, "ibv_query_port failed (errno ", errno, ")");
  }
  IbQpInfo info = {};
  info.lid = portAttr.lid;
  info.linkLayer = portAttr.link_layer;
  info.mtu = portAttr.active_mtu;
  info.isGrh = (portAttr.flags & IBV_QPF_GRH_REQUIRED);
  if (portAttr.link_layer != IBV_LINK_LAYER_INFINIBAND || info.isGrh) {
    if (gidIndex < 0 || gidIndex >= portAttr.gid_tbl_len) {
      THROW(NET, Error, ErrorCode::InvalidUsage, "invalid GID index ", gidIndex, " for port ", port, " (max index is ",
            portAttr.gid_tbl_len - 1, ")");
    }
    union ibv_gid gid = {};
    if (Verbs::ibv_query_gid(verbsCtx_, port, gidIndex, &gid) != 0) {
      THROW(NET, IbError, errno, "ibv_query_gid failed for port ", port, " index ", gidIndex, " (errno ", errno, ")");
    }
    info.spn = gid.global.subnet_prefix;
    info.iid = gid.global.interface_id;
  }
  return info;
}

template <typename Backend>
std::unique_ptr<const IbMr> VerbsCtx<Backend>::registerMr(void* buff, std::size_t size) {
  return std::unique_ptr<const IbMr>(new IbMr(pd_, buff, size, isDataDirect_, Backend::nativeMrc));
}

template <typename Backend>
bool VerbsCtx<Backend>::supportsRdmaAtomics() const {
  return supportsRdmaAtomics_;
}

template <typename Backend>
bool VerbsCtx<Backend>::isMlx5() const {
  return isMlx5_;
}

template <typename Backend>
bool VerbsCtx<Backend>::isDataDirect() const {
  return isDataDirect_;
}

template <typename Backend>
bool VerbsCtx<Backend>::isVirtualFunction() const {
  return isVF_;
}

MSCCLPP_API_CPP int getIBDeviceCount() {
  int num = 0;
  struct ibv_device** devices = IBVerbs::ibv_get_device_list(&num);
  if (devices == nullptr) return 0;
  IBVerbs::ibv_free_device_list(devices);
  return num;
}

std::string getHcaDevices(int deviceIndex) {
  std::string envStr = env()->hcaDevices;
  if (envStr != "") {
    std::vector<std::string> devices;
    std::stringstream ss(envStr);
    std::string device;
    while (std::getline(ss, device, ',')) {
      devices.push_back(device);
    }
    if (deviceIndex >= (int)devices.size()) {
      THROW(NET, Error, ErrorCode::InvalidUsage,
            "Not enough HCA devices are defined with MSCCLPP_HCA_DEVICES: ", envStr);
    }
    return devices[deviceIndex];
  }
  return "";
}

template <typename Backend>
static std::string getVerbsDeviceName(Transport transport) {
  using Verbs = typename Backend::Verbs;
  const auto transports = Backend::nativeMrc ? AllMRCTransports : AllIBTransports;
  if (!transports.has(transport)) {
    THROW(NET, Error, ErrorCode::InvalidUsage, "Not an ", Backend::name, " transport");
  }
  int deviceIndex = static_cast<int>(transport) - static_cast<int>(Backend::firstTransport);
  std::string userHcaDevice = getHcaDevices(deviceIndex);
  if (!userHcaDevice.empty()) {
    return userHcaDevice;
  }

  int num = 0;
  struct ibv_device** devices = Verbs::ibv_get_device_list(&num);
  auto freeDevices = [](ibv_device** devices) { Verbs::ibv_free_device_list(devices); };
  std::unique_ptr<ibv_device*, decltype(freeDevices)> deviceList(devices, freeDevices);
  if (!devices || deviceIndex >= num) {
    THROW(NET, Error, ErrorCode::InvalidUsage, Backend::name, " transport out of range: ", deviceIndex, " >= ", num);
  }
  return devices[deviceIndex]->name;
}

MSCCLPP_API_CPP std::string getIBDeviceName(Transport ibTransport) { return getVerbsDeviceName<Ib>(ibTransport); }

MSCCLPP_API_CPP Transport getIBTransportByDeviceName(const std::string& ibDeviceName) {
  int num;
  struct ibv_device** devices = IBVerbs::ibv_get_device_list(&num);
  for (int i = 0; i < num; ++i) {
    if (ibDeviceName == devices[i]->name) {
      switch (i) {  // TODO: get rid of this ugly switch
        case 0:
          return Transport::IB0;
        case 1:
          return Transport::IB1;
        case 2:
          return Transport::IB2;
        case 3:
          return Transport::IB3;
        case 4:
          return Transport::IB4;
        case 5:
          return Transport::IB5;
        case 6:
          return Transport::IB6;
        case 7:
          return Transport::IB7;
        default:
          THROW(NET, Error, ErrorCode::InvalidUsage, "IB device index out of range");
      }
    }
  }
  THROW(NET, Error, ErrorCode::InvalidUsage, "IB device not found");
}

#else  // !defined(USE_IBVERBS)

MSCCLPP_API_CPP int getIBDeviceCount() { return 0; }

MSCCLPP_API_CPP std::string getIBDeviceName(Transport) { return ""; }

MSCCLPP_API_CPP Transport getIBTransportByDeviceName(const std::string&) { return Transport::Unknown; }

IbMr::~IbMr() {}
IbMrInfo IbMr::getInfo() const { return IbMrInfo(); }
const void* IbMr::getBuff() const { return nullptr; }
uint32_t IbMr::getLkey() const { return 0; }

template <typename Backend>
VerbsQp<Backend>::~VerbsQp() {}
template <typename Backend>
void VerbsQp<Backend>::rtr(const IbQpInfo&) {}
template <typename Backend>
void VerbsQp<Backend>::rts() {}
template <typename Backend>
void VerbsQp<Backend>::stageSendWrite(const IbMr*, const IbMrInfo&, uint32_t, uint64_t, uint64_t, uint64_t, bool) {}
template <typename Backend>
void VerbsQp<Backend>::stageSendAtomicAdd(const IbMr*, const IbMrInfo&, uint64_t, uint64_t, uint64_t, bool) {}
template <typename Backend>
void VerbsQp<Backend>::stageSendWriteWithImm(const IbMr*, const IbMrInfo&, uint32_t, uint64_t, uint64_t, uint64_t, bool,
                                             unsigned int) {}
template <typename Backend>
void VerbsQp<Backend>::postSend() {}
template <typename Backend>
void VerbsQp<Backend>::stageRecv(uint64_t) {}
template <typename Backend>
void VerbsQp<Backend>::stageRecv(const IbMr*, uint64_t, uint32_t, uint64_t) {}
template <typename Backend>
void VerbsQp<Backend>::postRecv() {}
template <typename Backend>
int VerbsQp<Backend>::pollSendCq() {
  return 0;
}
template <typename Backend>
int VerbsQp<Backend>::pollRecvCq() {
  return 0;
}
template <typename Backend>
int VerbsQp<Backend>::getSendWcStatus(int) const {
  return 0;
}
template <typename Backend>
std::string VerbsQp<Backend>::getSendWcStatusString(int) const {
  return "";
}
template <typename Backend>
int VerbsQp<Backend>::getNumSendCqItems() const {
  return 0;
}
template <typename Backend>
int VerbsQp<Backend>::getRecvWcStatus(int) const {
  return 0;
}
template <typename Backend>
std::string VerbsQp<Backend>::getRecvWcStatusString(int) const {
  return "";
}
template <typename Backend>
unsigned int VerbsQp<Backend>::getRecvWcImmData(int) const {
  return 0;
}

template <typename Backend>
static std::string getVerbsDeviceName(Transport) {
  return "";
}

#endif  // !defined(USE_IBVERBS)

template <typename Backend>
class VerbsConnection : public BaseConnection {
 private:
  Transport transport_;
  Transport remoteTransport_;
  std::weak_ptr<VerbsQp<Backend>> qp_;
  std::unique_ptr<uint64_t> atomicSrc_;
  RegisteredMemory atomicSrcMem_;
  const IbMr* atomicSrcMr_ = nullptr;

  // For write-with-imm mode (HostNoAtomic): uses RDMA write-with-imm to signal
  // instead of atomic operations, with a host thread forwarding to GPU for memory consistency.
  bool ibNoAtomic_;
  bool gdrSignalForwarding_;  // ibNoAtomic_ && gdrEnabled() — decided once at construction
  std::thread recvThread_;
  std::atomic<bool> stopRecvThread_;
  std::atomic<bool> recvThreadError_;  // Set by recv thread on fatal error
  std::string recvThreadErrorMsg_;     // Error message from recv thread (written before recvThreadError_ is set)
  int localGpuDeviceId_;               // Local GPU device ID for CUDA context and GDR mapping

  // Signal forwarding design (HostNoAtomic mode):
  // - Sender: 0-byte RDMA WRITE_WITH_IMM carrying the lower 32 bits of the token in imm_data.
  // - Receiver: CPU recv thread polls recv CQ for WRITE_WITH_IMM completions (CQE), reads
  //   the lower 32 bits from imm_data, reconstructs the full 64-bit token using wrap-around
  //   detection (monotonically increasing tokens: if lower 32 bits decrease, the upper half
  //   incremented), then writes it to signalAddr_ via atomicStore through GDRCopy BAR1.
  uint64_t signalAddr_;

  std::unique_ptr<GdrMap> signalGdrMap_;

  void recvThreadFunc();

 public:
  VerbsConnection(std::shared_ptr<Context> context, const Endpoint& localEndpoint, const Endpoint& remoteEndpoint);
  ~VerbsConnection();

  /// Start signal forwarding to the given memory address.
  /// Must be called before the remote sends any updateAndSync in HostNoAtomic mode.
  /// @param mem Shared pointer to the GPU memory for the signal token.
  void startSignalForwarding(std::shared_ptr<uint64_t> mem) override;

  /// Stop signal forwarding and release associated resources.
  void stopSignalForwarding() override;

  bool isSignalForwarding() const override;

  Transport transport() const override;

  Transport remoteTransport() const override;

  void write(RegisteredMemory dst, uint64_t dstOffset, RegisteredMemory src, uint64_t srcOffset,
             uint64_t size) override;
  void updateAndSync(RegisteredMemory dst, uint64_t dstOffset, uint64_t* src, uint64_t newValue) override;
  void accumulate(RegisteredMemory dst, uint64_t dstOffset, int64_t value) override;

  void flush(int64_t timeoutUsec) override;

  void requestFlush() override;
  bool progressFlush() override;
};

IbMemory::IbMemory(std::shared_ptr<RdmaProvider> provider, std::unique_ptr<const IbMr> mr)
    : RdmaMemory(std::move(provider)), mr_(std::move(mr)), info_(mr_->getInfo()) {}

IbMemory::IbMemory(const IbMrInfo& info) { std::memcpy(&info_, &info, sizeof(info_)); }

void IbMemory::serialize(std::vector<char>& data) const { detail::serialize(data, info_); }

template <typename Backend>
VerbsEndpoint<Backend>::VerbsEndpoint(std::shared_ptr<RdmaProvider> provider, VerbsCtx<Backend>& context,
                                      EndpointConfig& config)
    : RdmaEndpoint(std::move(provider)), context_(&context), noAtomic_(Backend::nativeMrc) {
  if (config.maxWriteQueueSize <= 0) {
    config.maxWriteQueueSize = config.ib.maxCqSize;
  }

  if (config.ib.mode == EndpointConfig::Ib::Mode::Device) {
    if constexpr (Backend::nativeMrc) {
      THROW(NET, Error, ErrorCode::InvalidUsage, "Native MRC does not support Device mode");
    } else {
      if (config.device.type != DeviceType::GPU) {
        THROW(NET, Error, ErrorCode::InvalidUsage, "IB Device mode requires a GPU endpoint");
      }
      if (config.ib.gidIndex < 0) {
        config.ib.gidIndex = env()->ibGidIndex;
      }
      gpuNetIoQp_ = std::make_shared<GpuNetIoQp>(context, config.device.id, config.ib.port, config.ib.gidIndex);
      std::memcpy(&info_, &gpuNetIoQp_->info(), sizeof(info_));
      return;
    }
  }

  if (config.ib.mode == EndpointConfig::Ib::Mode::HostNoAtomic) {
    noAtomic_ = true;
  } else if (config.ib.mode == EndpointConfig::Ib::Mode::Default) {
    noAtomic_ = Backend::nativeMrc || (env()->ibvMode == "host-no-atomic");
  }

  if (!noAtomic_ && !context.supportsRdmaAtomics()) {
    WARN(NET, "IB device ", context.getDevName(),
         " does not support RDMA atomics. Falling back to write-with-immediate mode (HostNoAtomic).");
    noAtomic_ = true;
  }

  if (config.ib.gidIndex < 0) {
    config.ib.gidIndex = env()->ibGidIndex;
  }

  int maxRecvWr = noAtomic_ ? config.ib.maxRecvWr : 0;
  qp_ = context.createQp(config.ib.port, config.ib.gidIndex, config.ib.maxCqSize, config.ib.maxCqPollNum,
                         config.ib.maxSendWr, maxRecvWr, config.ib.maxWrPerSend, noAtomic_);
  std::memcpy(&info_, &qp_->getInfo(), sizeof(info_));
}

template <typename Backend>
VerbsEndpoint<Backend>::VerbsEndpoint(const IbQpInfo& info, bool noAtomic) : noAtomic_(noAtomic) {
  std::memcpy(&info_, &info, sizeof(info_));
}

template <typename Backend>
void VerbsEndpoint<Backend>::serialize(std::vector<char>& data) const {
  detail::serialize(data, info_);
  detail::serialize(data, noAtomic_);
}

template <typename Backend>
std::shared_ptr<BaseConnection> VerbsEndpoint<Backend>::createConnection(std::shared_ptr<Context> context,
                                                                         const Endpoint& local,
                                                                         const Endpoint& remote) const {
  if (!isLocal()) {
    THROW(CONN, Error, ErrorCode::InvalidUsage, "Cannot create a connection from a remote ", Backend::name,
          " endpoint");
  }
  if constexpr (!Backend::nativeMrc) {
    if (gpuNetIoQp_) {
      return std::make_shared<GpuNetIoConnection>(std::move(context), local, remote);
    }
  }
  return std::make_shared<VerbsConnection<Backend>>(std::move(context), local, remote);
}

template <typename Backend>
VerbsProvider<Backend>::VerbsProvider(Transport transport) : context_(getVerbsDeviceName<Backend>(transport)) {}

template <typename Backend>
std::shared_ptr<RdmaEndpoint> VerbsProvider<Backend>::createEndpoint(EndpointConfig& config) {
  return std::make_shared<VerbsEndpoint<Backend>>(shared_from_this(), context_, config);
}

template <typename Backend>
std::shared_ptr<RdmaMemory> VerbsProvider<Backend>::registerMemory(void* data, size_t size) {
  return std::make_shared<IbMemory>(shared_from_this(), context_.registerMr(data, size));
}

template <typename Backend>
static const VerbsEndpoint<Backend>& getVerbsEndpoint(const RdmaEndpoint& endpoint) {
  const auto* verbsEndpoint = dynamic_cast<const VerbsEndpoint<Backend>*>(&endpoint);
  if (!verbsEndpoint) {
    THROW(CONN, Error, ErrorCode::InvalidUsage, "Expected an ", Backend::name, " endpoint");
  }
  return *verbsEndpoint;
}

const IbEndpoint& getIbEndpoint(const RdmaEndpoint& endpoint) { return getVerbsEndpoint<Ib>(endpoint); }

static const IbMemory& getVerbsMemory(const TransportInfo& info) {
  if (!getRdmaTransports().has(info.transport) || !info.rdmaMemory) {
    THROW(NET, Error, ErrorCode::InvalidUsage, "Expected verbs memory");
  }
  // Both native backends bind transport records to the same MR representation.
  return static_cast<const IbMemory&>(*info.rdmaMemory);
}

const IbMemory& getIbMemory(const TransportInfo& info) {
  if (!AllIBTransports.has(info.transport) || !info.rdmaMemory) {
    THROW(NET, Error, ErrorCode::InvalidUsage, "Expected IB memory");
  }
  return getVerbsMemory(info);
}

template <typename Backend>
static std::shared_ptr<RdmaEndpoint> deserializeVerbsEndpoint(std::vector<char>::const_iterator& it) {
  IbQpInfo info;
  bool noAtomic;
  it = detail::deserialize(it, info);
  it = detail::deserialize(it, noAtomic);
  return std::make_shared<VerbsEndpoint<Backend>>(info, noAtomic);
}

std::shared_ptr<RdmaEndpoint> deserializeIbEndpoint(std::vector<char>::const_iterator& it) {
  return deserializeVerbsEndpoint<Ib>(it);
}

std::shared_ptr<RdmaEndpoint> deserializeMrcEndpoint(std::vector<char>::const_iterator& it) {
  return deserializeVerbsEndpoint<Mrc>(it);
}

std::shared_ptr<RdmaMemory> deserializeIbMemory(std::vector<char>::const_iterator& it) {
  IbMrInfo info;
  it = detail::deserialize(it, info);
  return std::make_shared<IbMemory>(info);
}

// Shared host connection protocol for IB and native MRC.

template <typename Backend>
void VerbsConnection<Backend>::recvThreadFunc() {
  // Set the CUDA device context for this thread
  if (localGpuDeviceId_ >= 0) {
    cudaError_t err = cudaSetDevice(localGpuDeviceId_);
    if (err != cudaSuccess) {
      WARN(NET, "IBConnection recvThreadFunc: cudaSetDevice(", localGpuDeviceId_,
           ") failed: ", cudaGetErrorString(err));
      return;
    }
    // Bind this thread to the NUMA node of the local GPU for optimal memory access
    int deviceNumaNode = getDeviceNumaNode(localGpuDeviceId_);
    if (deviceNumaNode >= 0) {
      numaBind(deviceNumaNode);
    }
  }

  uint32_t lastImmData = 0;
  uint64_t immHighBits = 0;
  uint64_t newValueHost = 0;

  auto qp = qp_.lock();
  if (!qp) return;

  while (!stopRecvThread_.load(std::memory_order_relaxed)) {
    int wcNum = qp->pollRecvCq();
    if (wcNum < 0) {
      recvThreadErrorMsg_ = "pollRecvCq failed";
      recvThreadError_.store(true, std::memory_order_release);
      WARN(NET, "IBConnection recvThreadFunc: ", recvThreadErrorMsg_);
      break;
    }

    for (int i = 0; i < wcNum; ++i) {
      int status = qp->getRecvWcStatus(i);
      if (status != static_cast<int>(WsStatus::Success)) {
        // A failed recv WC typically means the QP entered error state (e.g., WR Flushed Error).
        // All remaining WRs will also fail — no recovery without QP recreation. Exit the thread
        // and set the error flag so the main thread can detect it.
        recvThreadErrorMsg_ = std::string("recv work completion failed: ") + qp->getRecvWcStatusString(i);
        recvThreadError_.store(true, std::memory_order_release);
        WARN(NET, "IBConnection recvThreadFunc: ", recvThreadErrorMsg_);
        return;
      }

      // Read the lower 32 bits of the token from imm_data. Reconstruct the full 64-bit value
      // using wrap-around detection: tokens increase monotonically, so if the new lower 32 bits
      // are less than the previous value, the upper 32 bits must have incremented by 1.
      uint32_t immData = qp->getRecvWcImmData(i);
      if (immData < lastImmData) {
        immHighBits += (1ULL << 32);
      }
      lastImmData = immData;
      newValueHost = immHighBits | static_cast<uint64_t>(immData);

      // Forward the token to the semaphore's inbound token address via atomicStore
      // through the GDRCopy BAR1 mapping. The GPU reads with system-scope acquire.
      if (signalAddr_ != 0) {
        if (signalGdrMap_ && signalGdrMap_->valid()) {
          atomicStore(signalGdrMap_->hostPtr(), newValueHost, memoryOrderRelaxed);
        } else {
          // For HIP/ROCm.
          // NOTE: may need a fix in the future to ensure BAR1 mapping.
          *reinterpret_cast<volatile uint64_t*>(signalAddr_) = newValueHost;
        }
      }

      // Post another recv for future messages
      qp->stageRecv(/*wrId=*/0);
      qp->postRecv();
    }
  }
}

template <typename Backend>
VerbsConnection<Backend>::VerbsConnection(std::shared_ptr<Context> context, const Endpoint& localEndpoint,
                                          const Endpoint& remoteEndpoint)
    : BaseConnection(context, localEndpoint),
      transport_(localEndpoint.transport()),
      remoteTransport_(remoteEndpoint.transport()),
      atomicSrc_(std::make_unique<uint64_t>(0)),
      ibNoAtomic_(false),
      gdrSignalForwarding_(false),
      stopRecvThread_(false),
      recvThreadError_(false),
      localGpuDeviceId_(localEndpoint.device().id),
      signalAddr_(0) {
  const auto& localIbEndpoint = getVerbsEndpoint<Backend>(*getImpl(localEndpoint).rdmaEndpoint_);
  const auto& remoteIbEndpoint = getVerbsEndpoint<Backend>(*getImpl(remoteEndpoint).rdmaEndpoint_);
  ibNoAtomic_ = localIbEndpoint.noAtomic();
  if constexpr (Backend::nativeMrc) {
    if (!ibNoAtomic_ || !remoteIbEndpoint.noAtomic()) {
      THROW(CONN, Error, ErrorCode::InvalidUsage, "Native MRC requires write-with-immediate signaling on both peers");
    }
  }
  qp_ = localIbEndpoint.qp();
  qp_.lock()->rtr(remoteIbEndpoint.info());
  qp_.lock()->rts();
  atomicSrcMem_ = context->registerMemory(atomicSrc_.get(), sizeof(uint64_t), transport_);
  validateTransport(atomicSrcMem_, transport_);
  atomicSrcMr_ = getVerbsMemory(getImpl(atomicSrcMem_).getTransportInfo(transport_)).mr();

  if (ibNoAtomic_) {
#if defined(MSCCLPP_USE_CUDA)
    // On CUDA, HostNoAtomic requires GDRCopy for CPU→GPU signal forwarding through BAR1.
    if (!gdrEnabled()) {
      THROW(CONN, Error, ErrorCode::InvalidUsage,
            "IB host-no-atomic mode on CUDA requires GDRCopy: ", gdrStatusMessage());
    }
    gdrSignalForwarding_ = true;
#endif  // defined(MSCCLPP_USE_CUDA)

    // On platforms with a CPU-GPU bridge that reorders posted writes (e.g., Grace/GB200
    // NVLink-C2C), HostNoAtomic requires Data Direct for correct memory ordering. Data Direct
    // routes NIC DMA through the PCIe Data Direct engine, bypassing the bridge. It is available
    // on Virtual Function (VF) devices. On platforms without such a bridge (x86, non-Grace
    // aarch64), HostNoAtomic works without Data Direct.
    //
    // We cannot reliably detect the bridge at compile time or runtime, so we emit a warning
    // when the device is not a VF. If data corruption occurs, switching to VF devices with
    // Data Direct or using IbMode::Host with RDMA atomics will resolve it.
    {
      const auto& ibCtx = localIbEndpoint.context();
      if (!ibCtx.isVirtualFunction()) {
        WARN(CONN,
             "IB HostNoAtomic mode without a Virtual Function (VF) device may cause data corruption "
             "on platforms with a CPU-GPU bridge that reorders posted writes (e.g., Grace/GB200). "
             "Device ",
             ibCtx.getDevName(),
             " is not a VF. "
             "If you experience data corruption, use VF devices with Data Direct or IbMode::Host.");
      }
    }

    // Pre-post receive requests for incoming WRITE_WITH_IMM notifications.
    // The recv CQE guarantees the preceding data WRITE has been committed to GPU memory.
    auto qp = qp_.lock();
    int maxRecvWr = localEndpoint.config().ib.maxRecvWr;
    for (int i = 0; i < maxRecvWr; ++i) {
      qp->stageRecv(/*wrId=*/0);
    }
    qp->postRecv();
    // The recv thread is started later in startSignalForwarding() when the semaphore
    // provides the signal forwarding destination. This ensures the thread lifetime is
    // bounded by the GdrMap lifetime (created before start, destroyed after stop).
    INFO(CONN, Backend::name, "Connection via ", localIbEndpoint.context().getDevName(),
         " created with signal forwarding (HostNoAtomic) mode");
  } else {
    INFO(CONN, Backend::name, "Connection via ", localIbEndpoint.context().getDevName(), " created with atomic mode");
  }
}

template <typename Backend>
VerbsConnection<Backend>::~VerbsConnection() {
  stopSignalForwarding();
}

template <typename Backend>
Transport VerbsConnection<Backend>::transport() const {
  return transport_;
}

template <typename Backend>
Transport VerbsConnection<Backend>::remoteTransport() const {
  return remoteTransport_;
}

template <typename Backend>
bool VerbsConnection<Backend>::isSignalForwarding() const {
  return ibNoAtomic_;
}

template <typename Backend>
void VerbsConnection<Backend>::startSignalForwarding(std::shared_ptr<uint64_t> mem) {
  // Set up the forwarding destination and GdrMap, then start the recv thread.
  // Order: set address → create GdrMap → start thread.
  signalAddr_ = reinterpret_cast<uint64_t>(mem.get());
  if (gdrSignalForwarding_) {
    signalGdrMap_ = std::make_unique<GdrMap>(std::move(mem), localGpuDeviceId_);
  }
  if (ibNoAtomic_) {
    stopRecvThread_.store(false, std::memory_order_relaxed);
    recvThread_ = std::thread([this]() { this->recvThreadFunc(); });
  }
  INFO(CONN, "IBConnection startSignalForwarding: ", (void*)signalAddr_);
}

template <typename Backend>
void VerbsConnection<Backend>::stopSignalForwarding() {
  // Stop the recv thread, then tear down GdrMap and address.
  // Order: stop thread → destroy GdrMap → clear address.
  if (ibNoAtomic_) {
    stopRecvThread_.store(true, std::memory_order_relaxed);
    if (recvThread_.joinable()) {
      recvThread_.join();
    }
  }
  if (gdrSignalForwarding_) {
    signalGdrMap_.reset();
  }
  signalAddr_ = 0;
  INFO(CONN, "IBConnection stopSignalForwarding");
}

template <typename Backend>
void VerbsConnection<Backend>::write(RegisteredMemory dst, uint64_t dstOffset, RegisteredMemory src, uint64_t srcOffset,
                                     uint64_t size) {
#if defined(ENABLE_NPKIT) && defined(ENABLE_NPKIT_EVENT_CONN_IB_WRITE_ENTRY)
  NpKit::CollectCpuEvent(NPKIT_EVENT_CONN_IB_WRITE_ENTRY, uint32_t(size), 0, *NpKit::GetCpuTimestamp(), 0);
#endif

  validateTransport(dst, remoteTransport(), dstOffset, size);
  validateTransport(src, transport(), srcOffset, size);

  const auto& dstMemory = getVerbsMemory(getImpl(dst).getTransportInfo(remoteTransport()));
  if (dstMemory.isLocal()) {
    THROW(CONN, Error, ErrorCode::InvalidUsage, "dst is local, which is not supported");
  }
  const auto& srcMemory = getVerbsMemory(getImpl(src).getTransportInfo(transport()));
  if (!srcMemory.isLocal()) {
    THROW(CONN, Error, ErrorCode::InvalidUsage, "src is remote, which is not supported");
  }

  auto dstMrInfo = dstMemory.info();
  auto srcMr = srcMemory.mr();

  qp_.lock()->stageSendWrite(srcMr, dstMrInfo, (uint32_t)size, /*wrId=*/0, /*srcOffset=*/srcOffset,
                             /*dstOffset=*/dstOffset, /*signaled=*/true);

  qp_.lock()->postSend();
  INFO(CONN, "IBConnection write: from ", (uint8_t*)srcMr->getBuff() + srcOffset, " to ",
       (uint8_t*)dstMrInfo.addr + dstOffset, ", size ", size);

#if defined(ENABLE_NPKIT) && defined(ENABLE_NPKIT_EVENT_CONN_IB_WRITE_EXIT)
  NpKit::CollectCpuEvent(NPKIT_EVENT_CONN_IB_WRITE_EXIT, uint32_t(size), 0, *NpKit::GetCpuTimestamp(), 0);
#endif
}

template <typename Backend>
void VerbsConnection<Backend>::updateAndSync(RegisteredMemory dst, uint64_t dstOffset, uint64_t* src,
                                             uint64_t newValue) {
#if defined(ENABLE_NPKIT) && defined(ENABLE_NPKIT_EVENT_CONN_IB_UPDATE_AND_SYNC_ENTRY)
  NpKit::CollectCpuEvent(NPKIT_EVENT_CONN_IB_UPDATE_AND_SYNC_ENTRY, 0, 0, *NpKit::GetCpuTimestamp(), 0);
#endif

  validateTransport(dst, remoteTransport());
  const auto& dstMemory = getVerbsMemory(getImpl(dst).getTransportInfo(remoteTransport()));
  if (dstMemory.isLocal()) {
    THROW(CONN, Error, ErrorCode::InvalidUsage, "dst is local, which is not supported");
  }

  auto dstMrInfo = dstMemory.info();
  // assert that src is on host
  uint64_t oldValue = *src;
  *src = newValue;

  if (ibNoAtomic_) {
    // Signal forwarding: send a 0-byte RDMA WRITE_WITH_IMM with the lower 32 bits of the
    // token in imm_data. The receiver reconstructs the full 64-bit value using wrap-around
    // detection (tokens are monotonically increasing, so a decrease in the lower 32 bits
    // indicates the upper 32 bits incremented by 1).
    if (newValue <= oldValue) {
      WARN(CONN, "IBConnection signal forwarding: token is not monotonically increasing: ", oldValue, " -> ", newValue);
    } else if (newValue - oldValue >= (1ULL << 32)) {
      WARN(CONN,
           "IBConnection signal forwarding: token increment too large for 32-bit wrap-around detection: ", oldValue,
           " -> ", newValue, " (delta ", newValue - oldValue, " >= 2^32)");
    }
    unsigned int immData = static_cast<unsigned int>(newValue);
    qp_.lock()->stageSendWriteWithImm(nullptr, dstMrInfo,
                                      /*size=*/0, /*wrId=*/0,
                                      /*srcOffset=*/0, /*dstOffset=*/0,
                                      /*signaled=*/true, /*immData=*/immData);
    qp_.lock()->postSend();
    INFO(CONN, "IBConnection signal forwarding: value ", oldValue, " -> ", newValue);
  } else {
    qp_.lock()->stageSendAtomicAdd(atomicSrcMr_, dstMrInfo, /*wrId=*/0, dstOffset, newValue - oldValue,
                                   /*signaled=*/true);
    qp_.lock()->postSend();
    INFO(CONN, "IBConnection atomic write: from ", src, " to ", (uint8_t*)dstMrInfo.addr + dstOffset, ", ", oldValue,
         " -> ", newValue);
  }

#if defined(ENABLE_NPKIT) && defined(ENABLE_NPKIT_EVENT_CONN_IB_UPDATE_AND_SYNC_EXIT)
  NpKit::CollectCpuEvent(NPKIT_EVENT_CONN_IB_UPDATE_AND_SYNC_EXIT, 0, 0, *NpKit::GetCpuTimestamp(), 0);
#endif
}

template <typename Backend>
void VerbsConnection<Backend>::flush(int64_t timeoutUsec) {
#if defined(ENABLE_NPKIT) && defined(ENABLE_NPKIT_EVENT_CONN_IB_FLUSH_ENTRY)
  NpKit::CollectCpuEvent(NPKIT_EVENT_CONN_IB_FLUSH_ENTRY, 0, 0, *NpKit::GetCpuTimestamp(), 0);
#endif

  // Check if the recv thread has already reported an error (e.g., QP entered error state).
  if (recvThreadError_.load(std::memory_order_acquire)) {
    THROW(CONN, Error, ErrorCode::SystemError, "IBConnection recv thread failed: ", recvThreadErrorMsg_);
  }

  Timer timer;
  while (qp_.lock()->getNumSendCqItems()) {
    int wcNum = qp_.lock()->pollSendCq();
    if (wcNum < 0) {
      THROW(NET, IbError, errno, "pollSendCq failed");
    } else if (timeoutUsec >= 0) {
      auto elapsed = timer.elapsed();
      if (elapsed > timeoutUsec) {
        THROW(CONN, Error, ErrorCode::Timeout, "pollSendCq timed out: waited for ", elapsed / 1e6,
              " seconds. Expected ", qp_.lock()->getNumSendCqItems(), " signals");
      }
    }
    for (int i = 0; i < wcNum; ++i) {
      int status = qp_.lock()->getSendWcStatus(i);
      if (status != static_cast<int>(WsStatus::Success)) {
        THROW(NET, Error, ErrorCode::SystemError, "an IB work item failed: ", qp_.lock()->getSendWcStatusString(i));
      }
    }
  }
  INFO(CONN, "IBConnection flushing connection");

#if defined(ENABLE_NPKIT) && defined(ENABLE_NPKIT_EVENT_CONN_IB_FLUSH_EXIT)
  NpKit::CollectCpuEvent(NPKIT_EVENT_CONN_IB_FLUSH_EXIT, 0, 0, *NpKit::GetCpuTimestamp(), 0);
#endif
}

template <typename Backend>
void VerbsConnection<Backend>::accumulate(RegisteredMemory dst, uint64_t dstOffset, int64_t value) {
  validateTransport(dst, remoteTransport());
  validateAccumulateBounds(dst, dstOffset);
  const auto& dstMemory = getVerbsMemory(getImpl(dst).getTransportInfo(remoteTransport()));
  if (dstMemory.isLocal()) {
    THROW(CONN, Error, ErrorCode::InvalidUsage, "dst is local, which is not supported");
  }
  auto dstMrInfo = dstMemory.info();
  validateAccumulateAlignment(static_cast<uintptr_t>(dstMrInfo.addr), dstOffset);

  if (ibNoAtomic_) {
    THROW(CONN, Error, ErrorCode::InvalidUsage, "accumulate is not supported in ", Backend::name, " no-atomic mode");
  }

  qp_.lock()->stageSendAtomicAdd(atomicSrcMr_, dstMrInfo, /*wrId=*/0, dstOffset, static_cast<uint64_t>(value),
                                 /*signaled=*/true);
  qp_.lock()->postSend();
  INFO(CONN, "IBConnection accumulate: dst ", (uint8_t*)dstMrInfo.addr + dstOffset, ", value ", value);
}

template <typename Backend>
void VerbsConnection<Backend>::requestFlush() {
  // No-op: IB sends were already posted by prior conn.write() calls in handleTrigger.
  // progressFlush() drives completion by polling the send CQ.
}

template <typename Backend>
bool VerbsConnection<Backend>::progressFlush() {
  if (recvThreadError_.load(std::memory_order_acquire)) {
    THROW(CONN, Error, ErrorCode::SystemError, "IBConnection recv thread failed: ", recvThreadErrorMsg_);
  }

  auto qp = qp_.lock();
  if (!qp || qp->getNumSendCqItems() == 0) {
    return true;  // QP expired or CQ already drained.
  }

  int wcNum = qp->pollSendCq();
  if (wcNum < 0) {
    THROW(NET, IbError, errno, "pollSendCq failed in progressFlush");
  }
  for (int i = 0; i < wcNum; ++i) {
    int status = qp->getSendWcStatus(i);
    if (status != static_cast<int>(WsStatus::Success)) {
      THROW(NET, Error, ErrorCode::SystemError,
            "an IB work item failed in progressFlush: ", qp->getSendWcStatusString(i));
    }
  }
  return qp->getNumSendCqItems() == 0;
}

template class VerbsQp<Ib>;
template class VerbsQp<Mrc>;
template class VerbsCtx<Ib>;
template class VerbsCtx<Mrc>;
template class VerbsEndpoint<Ib>;
template class VerbsEndpoint<Mrc>;
template class VerbsProvider<Ib>;
template class VerbsProvider<Mrc>;

}  // namespace mscclpp
