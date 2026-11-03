// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#ifndef MSCCLPP_MRC_HPP_
#define MSCCLPP_MRC_HPP_

#if defined(MSCCLPP_USE_MRC)

#include <cstdint>
#include <memory>
#include <string>

struct ibv_context;
struct ibv_pd;
struct mrc_context;
struct mrc_cq;
struct mrc_qp;

namespace mscclpp {

// QP info to be shared with the remote peer.
struct MrcQpInfo {
  uint16_t lid;
  uint8_t linkLayer;
  uint32_t qpn;
  uint64_t spn;
  int mtu;
  uint64_t iid;
  bool isGrh;
};

class MrcQp {
 public:
  ~MrcQp();
  MrcQp(const MrcQp&) = delete;
  MrcQp& operator=(const MrcQp&) = delete;

  MrcQpInfo& getInfo() { return info_; }

 private:
  MrcQp(ibv_context* context, mrc_context* mrcContext, ibv_pd* pd, int port, int gidIndex, uint64_t evProfileId,
        int maxSendCqSize, int maxSendWr, int maxRecvWr);

  int portNum_;
  int gidIndex_;
  MrcQpInfo info_;

  mrc_cq* sendCq_;
  mrc_cq* recvCq_;
  mrc_qp* qp_;

  friend class MrcCtx;
};

class MrcCtx {
 public:
  explicit MrcCtx(const std::string& devName);
  ~MrcCtx();
  MrcCtx(const MrcCtx&) = delete;
  MrcCtx& operator=(const MrcCtx&) = delete;

  // The installed provider requires an explicit port for INIT, although mrc.h lists IBV_QP_PORT as unsupported.
  // The EV profile must already be configured by the controller.
  std::shared_ptr<MrcQp> createQp(int port, int gidIndex, uint64_t evProfileId, int maxSendCqSize, int maxSendWr,
                                  int maxRecvWr);

  const std::string& getDevName() const { return devName_; }

 private:
  const std::string devName_;
  ibv_context* context_;
  ibv_pd* pd_;
  mrc_context* mrcContext_;
};

}  // namespace mscclpp

#endif  // defined(MSCCLPP_USE_MRC)
#endif  // MSCCLPP_MRC_HPP_
