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
  qpAttrs.retry_cnt = 7;  // FIXME: should this be set dynamically based on some environment variable?
  qpAttrs.rnr_retry = 7;  // FIXME: should this be set dynamically based on some environment variable?

  mrc_qp_attr mrcAttrs{};
  mrcAttrs.timeout = 14;
  int ret = mrc_modify_qp(qp_, &qpAttrs, IBV_QP_STATE | IBV_QP_SQ_PSN | IBV_QP_RETRY_CNT | IBV_QP_RNR_RETRY, &mrcAttrs,
                          MRC_QP_TIMEOUT);
  if (ret != 0) {
    throw IbError("mrc_modify_qp failed transitioning to RTS", ret > 0 ? ret : errno);
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

std::shared_ptr<MrcQp> MrcCtx::createQp(int port, int gidIndex, uint64_t evProfileId, int maxSendCqSize, int maxSendWr,
                                        int maxRecvWr) {
  return std::shared_ptr<MrcQp>(
      new MrcQp(context_, mrcContext_, pd_, port, gidIndex, evProfileId, maxSendCqSize, maxSendWr, maxRecvWr));
}

}  // namespace mscclpp

#endif  // defined(MSCCLPP_USE_MRC)
