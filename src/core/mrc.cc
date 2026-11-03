// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#include "mrc.hpp"

#if defined(MSCCLPP_USE_MRC)

#include <mrc.h>

#include <cerrno>
#include <mscclpp/errors.hpp>

namespace mscclpp {

static_assert(MRC_API_CURRENT_VERSION == MRC_API_VER(1, 0, 0));
static_assert(MRC_API_VER_USED == MRC_API_VER(1, 0, 0));

MrcQp::MrcQp(mrc_context* context, ibv_pd* pd, int port, uint64_t evProfileId, int maxSendCqSize, int maxSendWr,
             int maxRecvWr)
    : sendCq_(nullptr), recvCq_(nullptr), qp_(nullptr) {
  if (maxSendCqSize <= 0 || maxSendWr <= 0 || maxRecvWr < 0) {
    throw Error("Native MRC requires positive send CQ/WR limits and a nonnegative receive WR limit",
                ErrorCode::InvalidUsage);
  }
  if (port <= 0 || port > 255) {
    throw Error("Native MRC requires a port between 1 and 255", ErrorCode::InvalidUsage);
  }

  std::unique_ptr<mrc_cq, decltype(&mrc_destroy_cq)> sendCq(mrc_create_cq(context, maxSendCqSize, nullptr, nullptr, 0),
                                                            mrc_destroy_cq);
  if (!sendCq) {
    throw IbError("mrc_create_cq failed for send CQ", errno);
  }

  std::unique_ptr<mrc_cq, decltype(&mrc_destroy_cq)> recvCq(nullptr, mrc_destroy_cq);
  if (maxRecvWr > 0) {
    recvCq.reset(mrc_create_cq(context, maxRecvWr, nullptr, nullptr, 0));
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
  std::unique_ptr<mrc_qp, decltype(&mrc_destroy_qp)> qp(mrc_create_qp(context, &attrs), mrc_destroy_qp);
  if (!qp) {
    throw IbError("mrc_create_qp failed", errno);
  }

  ibv_qp_attr qpAttrs{};
  qpAttrs.qp_state = IBV_QPS_INIT;
  qpAttrs.port_num = port;
  qpAttrs.pkey_index = 0;
  qpAttrs.qp_access_flags = IBV_ACCESS_REMOTE_WRITE;

  mrc_qp_attr mrcAttrs{};
  mrcAttrs.ev_profile_id = evProfileId;
  int ret = mrc_modify_qp(qp.get(), &qpAttrs, IBV_QP_STATE | IBV_QP_PORT | IBV_QP_PKEY_INDEX | IBV_QP_ACCESS_FLAGS,
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

MrcCtx::MrcCtx(const std::string& devName) : devName_(devName), context_(nullptr), pd_(nullptr), mrcContext_(nullptr) {
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

std::shared_ptr<MrcQp> MrcCtx::createQp(int port, uint64_t evProfileId, int maxSendCqSize, int maxSendWr,
                                        int maxRecvWr) {
  return std::shared_ptr<MrcQp>(new MrcQp(mrcContext_, pd_, port, evProfileId, maxSendCqSize, maxSendWr, maxRecvWr));
}

}  // namespace mscclpp

#endif  // defined(MSCCLPP_USE_MRC)
