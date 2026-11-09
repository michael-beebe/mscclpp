// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#include <array>
#include <cstring>
#include <mscclpp/core.hpp>
#include <sstream>

#include "../framework.hpp"
#include "connection.hpp"
#include "serialization.hpp"
#include "utils_internal.hpp"

namespace {

struct RdmaTestAccess : mscclpp::BaseConnection {
  using BaseConnection::getImpl;
};

}  // namespace

class LocalCommunicatorTest : public ::mscclpp::test::TestCase {
 protected:
  void SetUp() override {
    bootstrap = std::make_shared<mscclpp::TcpBootstrap>(0, 1);
    bootstrap->initialize(bootstrap->createUniqueId());
    comm = std::make_shared<mscclpp::Communicator>(bootstrap);
  }

  std::shared_ptr<mscclpp::TcpBootstrap> bootstrap;
  std::shared_ptr<mscclpp::Communicator> comm;
};

TEST(TransportFlagsTest, OutputStream) {
  auto toString = [](const mscclpp::TransportFlags& transportFlags) {
    std::ostringstream os;
    os << transportFlags;
    return os.str();
  };

  EXPECT_EQ(toString(mscclpp::NoTransports), std::string("{}"));
  EXPECT_EQ(toString(mscclpp::Transport::CudaIpc), std::string("{IPC}"));
  EXPECT_EQ(toString(mscclpp::Transport::CudaIpc | mscclpp::Transport::IB0 | mscclpp::Transport::Ethernet),
            std::string("{IPC, IB0, ETH}"));
}

TEST(LocalCommunicatorTest, RegisterMemory) {
  int dummy[42];
  auto memory = comm->registerMemory(&dummy, sizeof(dummy), mscclpp::NoTransports);
  EXPECT_EQ(memory.data(), &dummy);
  EXPECT_EQ(memory.size(), sizeof(dummy));
  EXPECT_EQ(memory.transports(), mscclpp::NoTransports);
}

TEST(LocalCommunicatorTest, SendMemoryToSelf) {
  int dummy[42];
  auto memory = comm->registerMemory(&dummy, sizeof(dummy), mscclpp::NoTransports);
  comm->sendMemory(memory, 0);
  auto memoryFuture = comm->recvMemory(0);
  auto sameMemory = memoryFuture.get();
  EXPECT_EQ(sameMemory.data(), memory.data());
  EXPECT_EQ(sameMemory.size(), memory.size());
  EXPECT_EQ(sameMemory.transports(), memory.transports());
}

TEST(RdmaProviderTest, ImportedEndpointPreservesWireBytes) {
  using namespace mscclpp;
  IbQpInfo info;
  std::memset(&info, 0xa5, sizeof(info));
  info.lid = 17;
  info.linkLayer = 1;
  info.qpn = 23;
  info.spn = 31;
  info.mtu = 4;
  info.iid = 41;
  info.isGrh = true;
  for (bool noAtomic : {false, true}) {
    EndpointConfig config(Transport::IB7, DeviceType::CPU, 13);
    config.ib.mode = noAtomic ? EndpointConfig::Ib::Mode::HostNoAtomic : EndpointConfig::Ib::Mode::Host;
    config.ib.gidIndex = 3;
    uint64_t hostHash = 43;
    uint64_t pidHash = 47;
    std::vector<char> bytes;
    detail::serialize(bytes, config);
    detail::serialize(bytes, hostHash);
    detail::serialize(bytes, pidHash);
    detail::serialize(bytes, info);
    detail::serialize(bytes, noAtomic);

    auto endpoint = Endpoint::deserialize(bytes);
    EXPECT_TRUE(endpoint.serialize() == bytes);
    EXPECT_FALSE(RdmaTestAccess::getImpl(endpoint).rdmaEndpoint_->isLocal());
  }
}

TEST(RdmaProviderTest, ImportedMemoryPreservesRecordOrderAndBytes) {
  using namespace mscclpp;
  uint64_t buffer = 0;
  void* ptr = &buffer;
  size_t size = sizeof(buffer);
  uint64_t hostHash = getHostHash() ^ 1;
  uint64_t pidHash = getPidHash();
  TransportFlags transports = Transport::IB0 | Transport::IB1;
  int8_t transportCount = 3;
  std::vector<char> bytes;
  detail::serialize(bytes, ptr);
  detail::serialize(bytes, size);
  detail::serialize(bytes, hostHash);
  detail::serialize(bytes, pidHash);
  detail::serialize(bytes, transports);
  detail::serialize(bytes, transportCount);
  uint64_t address = 0x1000;
  for (auto transport : {Transport::IB1, Transport::IB0, Transport::IB1}) {
    IbMrInfo info;
    std::memset(&info, 0x5a, sizeof(info));
    info.addr = address;
    info.rkey = 53;
    detail::serialize(bytes, transport);
    detail::serialize(bytes, info);
    address += 0x100;
  }

  auto memory = RegisteredMemory::deserialize(bytes);
  EXPECT_TRUE(memory.serialize() == bytes);
  EXPECT_EQ(memory.data(), nullptr);
  const auto& firstIb1 = RdmaTestAccess::getImpl(memory).getTransportInfo(Transport::IB1);
  EXPECT_EQ(firstIb1.ibMrInfo.addr, uint64_t{0x1000});
  EXPECT_FALSE(firstIb1.ibLocal);
}

TEST(RdmaProviderTest, LocalResourcesRetainProvider) {
  using namespace mscclpp;
  if (getIBDeviceCount() == 0) {
    SKIP_TEST() << "Requires an IB device";
  }
  MSCCLPP_CUDATHROW(cudaSetDevice(0));
  for (bool memoryFirst : {false, true}) {
    std::array<uint64_t, 8> buffer{};
    auto context = Context::create();
    EndpointConfig::Ib ib;
    ib.maxCqSize = 8;
    ib.maxCqPollNum = 8;
    ib.maxSendWr = 16;
    ib.maxRecvWr = 8;
    ib.maxWrPerSend = 4;
    auto endpoint = context->createEndpoint({Transport::IB0, DeviceType::CPU, -1, ib});
    auto memory = context->registerMemory(buffer.data(), sizeof(buffer), Transport::IB0);
    EXPECT_EQ(RdmaTestAccess::getImpl(*context).rdmaProviders_.size(), size_t{1});
    std::weak_ptr<RdmaProvider> provider = RdmaTestAccess::getImpl(*context).rdmaProviders_.at(Transport::IB0);
    std::weak_ptr<Context> outerContext = context;
    auto endpointBytes = endpoint.serialize();
    auto memoryBytes = memory.serialize();

    context.reset();
    EXPECT_TRUE(outerContext.expired());
    ASSERT_FALSE(provider.expired());
    EXPECT_TRUE(endpoint.serialize() == endpointBytes);
    EXPECT_TRUE(memory.serialize() == memoryBytes);
    if (memoryFirst) {
      memory = RegisteredMemory();
    } else {
      endpoint = Endpoint();
    }
    ASSERT_FALSE(provider.expired());
    memory = RegisteredMemory();
    endpoint = Endpoint();
    EXPECT_TRUE(provider.expired());
  }
}
