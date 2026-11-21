// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#ifndef MSCCLPP_IB_HPP_
#define MSCCLPP_IB_HPP_

#include <list>
#include <memory>
#include <mscclpp/core.hpp>
#include <string>
#include <string_view>

#include "rdma_provider.hpp"

// Forward declarations of IB structures
struct ibv_context;
struct ibv_pd;
struct ibv_mr;
struct ibv_qp;
struct ibv_cq;
struct ibv_wc;
struct ibv_send_wr;
struct ibv_recv_wr;
struct ibv_sge;
struct ibv_qp_attr;
struct ibv_qp_cap;
struct mrc_context;
struct mrc_qp;
struct mrc_cq;
struct mrc_qp_hint;

namespace mscclpp {

struct TransportInfo;
struct GpuNetIoDevice;
class GpuNetIoQp;

template <bool Native>
struct VerbsLib;

// Only the context/CQ/QP API differs. Both backends use native verbs PDs, MRs,
// work requests and completions. MRC must not open a context through the shim.
struct Ib {
  struct Hint {};
  using Context = ibv_context;
  using Qp = ibv_qp;
  using Cq = ibv_cq;
  using Verbs = VerbsLib<false>;
  static constexpr bool nativeMrc = false;
  static constexpr std::string_view name = "IB";
  static constexpr Transport firstTransport = Transport::IB0;

  static Context* createContext(ibv_context* context);
  static void destroyContext(Context* context);
  static Cq* createCq(Context* context, int size);
  static void destroyCq(Cq* cq);
  static Hint* createHint(Context* context);
  static int destroyHint(Hint* hint);
  static Qp* createQp(Context* context, ibv_pd* pd, Cq* sendCq, Cq* recvCq, const ibv_qp_cap& cap);
  static int destroyQp(Qp* qp);
  static int getQpn(Qp* qp, uint32_t* qpn);
  static int modifyQp(Qp* qp, ibv_qp_attr* attr, int mask, Hint* hint = nullptr);
  static int postSend(Qp* qp, ibv_send_wr* wr, ibv_send_wr** badWr);
  static int postRecv(Qp* qp, ibv_recv_wr* wr, ibv_recv_wr** badWr);
  static int pollCq(Cq* cq, int count, ibv_wc* wc);
};

struct Mrc {
  using Hint = mrc_qp_hint;
  using Context = mrc_context;
  using Qp = mrc_qp;
  using Cq = mrc_cq;
  using Verbs = VerbsLib<true>;
  static constexpr bool nativeMrc = true;
  static constexpr std::string_view name = "MRC";
  static constexpr Transport firstTransport = Transport::MRC0;

  static Context* createContext(ibv_context* context);
  static void destroyContext(Context* context);
  static Cq* createCq(Context* context, int size);
  static void destroyCq(Cq* cq);
  static Hint* createHint(Context* context);
  static int destroyHint(Hint* hint);
  static Qp* createQp(Context* context, ibv_pd* pd, Cq* sendCq, Cq* recvCq, const ibv_qp_cap& cap);
  static int destroyQp(Qp* qp);
  static int getQpn(Qp* qp, uint32_t* qpn);
  static int modifyQp(Qp* qp, ibv_qp_attr* attr, int mask, Hint* hint = nullptr);
  static int postSend(Qp* qp, ibv_send_wr* wr, ibv_send_wr** badWr);
  static int postRecv(Qp* qp, ibv_recv_wr* wr, ibv_recv_wr** badWr);
  static int pollCq(Cq* cq, int count, ibv_wc* wc);
};

template <typename Backend>
class VerbsCtx;

struct IbMrInfo {
  uint64_t addr;
  uint32_t rkey;
};

class IbMr {
 public:
  ~IbMr();

  IbMrInfo getInfo() const;
  const void* getBuff() const;
  uint32_t getLkey() const;

 private:
  IbMr(ibv_pd* pd, void* buff, std::size_t size, bool isDataDirect, bool native);

  ibv_mr* mr_;
  void* buff_;
  std::size_t size_;
  bool native_;

  template <typename Backend>
  friend class VerbsCtx;
};

// QP info to be shared with the remote peer
struct IbQpInfo {
  uint16_t lid;
  uint8_t linkLayer;
  uint32_t qpn;
  uint64_t spn;
  int mtu;
  uint64_t iid;
  bool isGrh;
};

enum class WsStatus {
  Success,
};

template <typename Backend>
class VerbsQp {
 public:
  ~VerbsQp();

  void rtr(const IbQpInfo& info);
  void rts();
  void stageSendWrite(const IbMr* mr, const IbMrInfo& info, uint32_t size, uint64_t wrId, uint64_t srcOffset,
                      uint64_t dstOffset, bool signaled);
  void stageSendAtomicAdd(const IbMr* mr, const IbMrInfo& info, uint64_t wrId, uint64_t dstOffset, uint64_t addVal,
                          bool signaled);
  void stageSendWriteWithImm(const IbMr* mr, const IbMrInfo& info, uint32_t size, uint64_t wrId, uint64_t srcOffset,
                             uint64_t dstOffset, bool signaled, unsigned int immData);
  void postSend();

  void stageRecv(uint64_t wrId);
  void stageRecv(const IbMr* mr, uint64_t wrId, uint32_t size, uint64_t offset = 0);
  void postRecv();

  int pollSendCq();
  int pollRecvCq();

  IbQpInfo& getInfo() { return info_; }
  int getSendWcStatus(int idx) const;
  std::string getSendWcStatusString(int idx) const;
  int getNumSendCqItems() const;
  int getRecvWcStatus(int idx) const;
  std::string getRecvWcStatusString(int idx) const;
  unsigned int getRecvWcImmData(int idx) const;

 private:
  using Verbs = typename Backend::Verbs;

  struct SendWrInfo {
    ibv_send_wr* wr;
    ibv_sge* sge;
  };

  struct RecvWrInfo {
    ibv_recv_wr* wr;
    ibv_sge* sge;
  };

  VerbsQp(typename Backend::Context* ctx, ibv_context* verbsCtx, ibv_pd* pd, int portNum, int gidIndex,
          int maxSendCqSize, int maxSendCqPollNum, int maxSendWr, int maxRecvWr, int maxWrPerSend, bool noAtomic);
  void reset();
  SendWrInfo getNewSendWrInfo();
  RecvWrInfo getNewRecvWrInfo();

  int portNum_;
  int gidIndex_;

  IbQpInfo info_;

  typename Backend::Qp* qp_;
  typename Backend::Hint* hint_;
  typename Backend::Cq* sendCq_;
  typename Backend::Cq* recvCq_;
  std::shared_ptr<std::vector<ibv_wc>> sendWcs_;
  std::shared_ptr<std::vector<ibv_wc>> recvWcs_;
  std::shared_ptr<std::vector<ibv_send_wr>> sendWrs_;
  std::shared_ptr<std::vector<ibv_sge>> sendSges_;
  std::shared_ptr<std::vector<ibv_recv_wr>> recvWrs_;
  std::shared_ptr<std::vector<ibv_sge>> recvSges_;
  int numStagedSend_;
  int numStagedRecv_;
  int numPostedSignaledSend_;
  int numStagedSignaledSend_;

  const int maxSendCqSize_;
  const int maxSendCqPollNum_;
  const int maxSendWr_;
  const int maxWrPerSend_;
  const int maxRecvWr_;
  const bool noAtomic_;

  friend class VerbsCtx<Backend>;
};

template <typename Backend>
class VerbsCtx {
 public:
#if defined(USE_IBVERBS)
  VerbsCtx(const std::string& devName);
  ~VerbsCtx();

  std::shared_ptr<VerbsQp<Backend>> createQp(int port, int gidIndex, int maxSendCqSize, int maxSendCqPollNum,
                                             int maxSendWr, int maxRecvWr, int maxWrPerSend, bool noAtomic);
  std::unique_ptr<const IbMr> registerMr(void* buff, std::size_t size);
  bool supportsRdmaAtomics() const;
  bool isMlx5() const;
  bool isDataDirect() const;
  bool isVirtualFunction() const;

  /// Resolve `port` (or pick any usable port if it is -1) and return the local port/GID addressing info that a
  /// remote QP needs to connect to a QP on this port. `qpn` is left as 0.
  IbQpInfo getPortInfo(int& port, int gidIndex) const;
  ibv_pd* getPd() const { return pd_; }

  /// DOCA GPUNetIO handles opened on this device's protection domain, created on first use. Requires GPUNetIO.
  GpuNetIoDevice& gpuNetIoDevice();
#else
  VerbsCtx(const std::string& devName) : devName_(devName) {}
  ~VerbsCtx() {}

  std::shared_ptr<VerbsQp<Backend>> createQp(int, int, int, int, int, int, int, bool) { return nullptr; }
  std::unique_ptr<const IbMr> registerMr([[maybe_unused]] void* buff, [[maybe_unused]] std::size_t size) {
    return nullptr;
  }
  bool supportsRdmaAtomics() const { return false; }
  bool isMlx5() const { return false; }
  bool isDataDirect() const { return false; }
  bool isVirtualFunction() const { return false; }
  IbQpInfo getPortInfo(int&, int) const { return {}; }
  ibv_pd* getPd() const { return nullptr; }
#endif

  const std::string& getDevName() const { return devName_; };

 private:
  using Verbs = typename Backend::Verbs;
  void reset();
  bool isPortUsable(int port, int gidIndex) const;
  int getAnyUsablePort(int gidIndex) const;

  const std::string devName_;
  ibv_context* verbsCtx_;
  typename Backend::Context* ctx_;
  ibv_pd* pd_;
  bool supportsRdmaAtomics_;
  bool isMlx5_;
  bool isDataDirect_;
  bool isVF_;
#if defined(USE_IBVERBS)
  std::unique_ptr<GpuNetIoDevice> gpuNetIoDevice_;
#endif
};

using IbCtx = VerbsCtx<Ib>;
using MrcCtx = VerbsCtx<Mrc>;
using IbQp = VerbsQp<Ib>;
using MrcQp = VerbsQp<Mrc>;

#if defined(USE_IBVERBS)
template <>
GpuNetIoDevice& VerbsCtx<Ib>::gpuNetIoDevice();
#endif

// Registration and imported MR metadata are identical for native IB and MRC.
class IbMemory final : public RdmaMemory {
 public:
  IbMemory(std::shared_ptr<RdmaProvider> provider, std::unique_ptr<const IbMr> mr);
  explicit IbMemory(const IbMrInfo& info);
  bool isLocal() const override { return mr_ != nullptr; }
  void serialize(std::vector<char>& data) const override;
  const IbMr* mr() const { return mr_.get(); }
  const IbMrInfo& info() const { return info_; }

 private:
  std::unique_ptr<const IbMr> mr_;
  IbMrInfo info_;
};

template <typename Backend>
class VerbsEndpoint final : public RdmaEndpoint {
 public:
  VerbsEndpoint(std::shared_ptr<RdmaProvider> provider, VerbsCtx<Backend>& context, EndpointConfig& config);
  VerbsEndpoint(const IbQpInfo& info, bool noAtomic);
  bool isLocal() const override { return qp_ != nullptr || gpuNetIoQp_ != nullptr; }
  void serialize(std::vector<char>& data) const override;
  std::shared_ptr<BaseConnection> createConnection(std::shared_ptr<Context> context, const Endpoint& local,
                                                   const Endpoint& remote) const override;
  const std::shared_ptr<VerbsQp<Backend>>& qp() const { return qp_; }
  const std::shared_ptr<GpuNetIoQp>& gpuNetIoQp() const { return gpuNetIoQp_; }
  const IbQpInfo& info() const { return info_; }
  bool noAtomic() const { return noAtomic_; }
  const VerbsCtx<Backend>& context() const { return *context_; }

 private:
  VerbsCtx<Backend>* context_ = nullptr;
  std::shared_ptr<VerbsQp<Backend>> qp_;
  std::shared_ptr<GpuNetIoQp> gpuNetIoQp_;
  IbQpInfo info_;
  bool noAtomic_;
};

template <typename Backend>
class VerbsProvider final : public RdmaProvider {
 public:
  explicit VerbsProvider(Transport transport);
  std::shared_ptr<RdmaEndpoint> createEndpoint(EndpointConfig& config) override;
  std::shared_ptr<RdmaMemory> registerMemory(void* data, size_t size) override;

 private:
  VerbsCtx<Backend> context_;
};

using IbEndpoint = VerbsEndpoint<Ib>;
using MrcEndpoint = VerbsEndpoint<Mrc>;
using IbProvider = VerbsProvider<Ib>;
using MrcProvider = VerbsProvider<Mrc>;

const IbEndpoint& getIbEndpoint(const RdmaEndpoint& endpoint);
const IbMemory& getIbMemory(const TransportInfo& info);
std::shared_ptr<RdmaEndpoint> deserializeIbEndpoint(std::vector<char>::const_iterator& it);
std::shared_ptr<RdmaEndpoint> deserializeMrcEndpoint(std::vector<char>::const_iterator& it);
std::shared_ptr<RdmaMemory> deserializeIbMemory(std::vector<char>::const_iterator& it);

}  // namespace mscclpp

#endif  // MSCCLPP_IB_HPP_
