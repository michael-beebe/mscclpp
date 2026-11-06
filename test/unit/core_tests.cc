// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#include <mscclpp/core.hpp>
#include <mscclpp/errors.hpp>
#include <sstream>
#include <utility>

#include "../framework.hpp"
#include "serialization.hpp"

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
  EXPECT_EQ(toString(mscclpp::Transport::MRC0 | mscclpp::Transport::MRC3), std::string("{MRC0, MRC3}"));
}

TEST(TransportFlagsTest, MrcIsExplicitOptIn) {
  EXPECT_TRUE((mscclpp::AllTransports & mscclpp::AllMrcTransports).none());
  EXPECT_TRUE((mscclpp::AllIBTransports & mscclpp::AllMrcTransports).none());
  EXPECT_TRUE(mscclpp::AllTransports ==
              (mscclpp::AllIBTransports | mscclpp::Transport::CudaIpc | mscclpp::Transport::Ethernet));
  auto selected = mscclpp::Transport::CudaIpc | mscclpp::Transport::MRC1;
  EXPECT_TRUE(selected.has(mscclpp::Transport::MRC1));
  EXPECT_FALSE(selected.has(mscclpp::Transport::IB1));
  EXPECT_EQ((selected & mscclpp::AllMrcTransports).count(), size_t{1});
}

#if !defined(MSCCLPP_USE_IB)
TEST(IbDisabledTest, ExplicitRequestsFail) {
  auto context = mscclpp::Context::create();
  unsigned char buffer[64]{};
  auto expectDisabled = [](auto function) {
    bool caught = false;
    try {
      function();
    } catch (const mscclpp::Error& error) {
      EXPECT_TRUE(error.getErrorCode() == mscclpp::ErrorCode::InvalidUsage);
      EXPECT_TRUE(std::string(error.what()).find("IB support is disabled") != std::string::npos);
      caught = true;
    }
    EXPECT_TRUE(caught);
  };
  expectDisabled([&] { context->registerMemory(buffer, sizeof(buffer), mscclpp::Transport::IB0); });
  expectDisabled([&] { context->createEndpoint({mscclpp::Transport::IB0, mscclpp::DeviceType::CPU}); });
  expectDisabled([&] { context->createEndpoint({mscclpp::Transport::IB0}); });
}
#endif

#if !defined(MSCCLPP_USE_MRC)
TEST(MrcDisabledTest, ExplicitRequestsFail) {
  auto context = mscclpp::Context::create();
  unsigned char buffer[64]{};
  auto expectDisabled = [](auto function) {
    bool caught = false;
    try {
      function();
    } catch (const mscclpp::Error& error) {
      EXPECT_TRUE(error.getErrorCode() == mscclpp::ErrorCode::InvalidUsage);
      EXPECT_TRUE(std::string(error.what()).find("Native MRC support is disabled") != std::string::npos);
      caught = true;
    }
    EXPECT_TRUE(caught);
  };
  expectDisabled([&] { context->registerMemory(buffer, sizeof(buffer), mscclpp::Transport::MRC0); });
  expectDisabled([&] { context->createEndpoint({mscclpp::Transport::MRC0, mscclpp::DeviceType::CPU}); });
  expectDisabled([&] { context->createEndpoint({mscclpp::Transport::MRC0}); });
}
#endif

#if !defined(MSCCLPP_USE_IB) || !defined(MSCCLPP_USE_MRC)
TEST(DisabledTransportMetadataTest, RejectsDisabledBackends) {
  std::vector<std::pair<mscclpp::Transport, const char*>> disabled;
#if !defined(MSCCLPP_USE_IB)
  disabled.emplace_back(mscclpp::Transport::IB0, "IB support is disabled");
#endif
#if !defined(MSCCLPP_USE_MRC)
  disabled.emplace_back(mscclpp::Transport::MRC0, "Native MRC support is disabled");
#endif
  for (const auto& [transport, message] : disabled) {
    auto expectDisabled = [&](auto function) {
      bool caught = false;
      try {
        function();
      } catch (const mscclpp::Error& error) {
        EXPECT_TRUE(error.getErrorCode() == mscclpp::ErrorCode::InvalidUsage);
        EXPECT_TRUE(std::string(error.what()).find(message) != std::string::npos);
        caught = true;
      }
      EXPECT_TRUE(caught);
    };
    std::vector<char> endpoint;
    mscclpp::detail::serialize(endpoint, mscclpp::EndpointConfig(transport, mscclpp::DeviceType::CPU));
    mscclpp::detail::serialize(endpoint, uint64_t{1});
    mscclpp::detail::serialize(endpoint, uint64_t{2});
    expectDisabled([&] { mscclpp::Endpoint::deserialize(endpoint); });

    for (auto flags : {mscclpp::TransportFlags(transport), mscclpp::NoTransports}) {
      std::vector<char> memory;
      mscclpp::detail::serialize(memory, reinterpret_cast<void*>(uintptr_t{0x1000}));
      mscclpp::detail::serialize(memory, size_t{64});
      mscclpp::detail::serialize(memory, uint64_t{1});
      mscclpp::detail::serialize(memory, uint64_t{2});
      mscclpp::detail::serialize(memory, flags);
      int8_t count = flags.none() ? 1 : 0;
      mscclpp::detail::serialize(memory, count);
      if (count != 0) mscclpp::detail::serialize(memory, transport);
      expectDisabled([&] { mscclpp::RegisteredMemory::deserialize(memory); });
    }
  }
}
#endif

TEST(EndpointMetadataTest, ExistingTransportRoundtrip) {
  auto context = mscclpp::Context::create();
  auto endpoint = context->createEndpoint({mscclpp::Transport::CudaIpc, mscclpp::DeviceType::CPU});
  auto imported = mscclpp::Endpoint::deserialize(endpoint.serialize());
  EXPECT_TRUE(imported.transport() == mscclpp::Transport::CudaIpc);
  EXPECT_TRUE(imported.device().type == mscclpp::DeviceType::CPU);
  EXPECT_TRUE(imported.serialize() == endpoint.serialize());
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
