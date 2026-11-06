// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#include <mscclpp/core.hpp>
#include <mscclpp/errors.hpp>

#include "../framework.hpp"
#include "mrc.hpp"
#include "serialization.hpp"

class MrcMetadataTest : public mscclpp::test::TestCase {
 protected:
  template <typename F>
  void expectInvalid(F function) {
    bool caught = false;
    try {
      function();
    } catch (const mscclpp::Error& error) {
      EXPECT_TRUE(error.getErrorCode() == mscclpp::ErrorCode::InvalidUsage);
      caught = true;
    }
    EXPECT_TRUE(caught);
  }

  std::vector<char> endpointRecord() {
    mscclpp::EndpointConfig config(mscclpp::Transport::MRC1, mscclpp::DeviceType::CPU, 8, {}, {1, 3, 0, 8, 8, 8});
    std::vector<char> data;
    mscclpp::detail::serialize(data, config);
    mscclpp::detail::serialize(data, uint64_t{1});
    mscclpp::detail::serialize(data, uint64_t{2});
    mscclpp::detail::serialize(data, qpInfo);
    return data;
  }

  std::vector<char> memoryRecord(int8_t count = 1) {
    std::vector<char> data;
    mscclpp::detail::serialize(data, reinterpret_cast<void*>(mrInfo.addr));
    mscclpp::detail::serialize(data, size_t{64});
    mscclpp::detail::serialize(data, uint64_t{1});
    mscclpp::detail::serialize(data, uint64_t{2});
    mscclpp::detail::serialize(data, mscclpp::TransportFlags(mscclpp::Transport::MRC1));
    mscclpp::detail::serialize(data, count);
    for (int i = 0; i < count; ++i) {
      mscclpp::detail::serialize(data, mscclpp::Transport::MRC1);
      mscclpp::detail::serialize(data, mrInfo);
    }
    return data;
  }

  mscclpp::MrcQpInfo qpInfo{0x1234, 2, 0x123456, 0x0102030405060708, 3, 0x1122334455667788, true};
  mscclpp::MrcMrInfo mrInfo{0x0102030405060708, 0x90a0b0c0};
};

TEST(MrcMetadataTest, EndpointRoundtripAndConnectionGate) {
  auto data = endpointRecord();
  auto endpoint = mscclpp::Endpoint::deserialize(data);
  EXPECT_TRUE(endpoint.transport() == mscclpp::Transport::MRC1);
  EXPECT_EQ(endpoint.config().mrc.gidIndex, 3);
  EXPECT_TRUE(endpoint.serialize() == data);
  auto context = mscclpp::Context::create();
  expectInvalid([&] { context->connect(endpoint, endpoint); });
  for (size_t size = 0; size < data.size(); ++size) {
    expectInvalid([&] { mscclpp::Endpoint::deserialize(std::vector<char>(data.begin(), data.begin() + size)); });
  }
}

TEST(MrcMetadataTest, MemoryRoundtripAndRecordValidation) {
  auto data = memoryRecord();
  auto memory = mscclpp::RegisteredMemory::deserialize(data);
  EXPECT_EQ(memory.size(), size_t{64});
  EXPECT_TRUE(memory.transports() == mscclpp::TransportFlags(mscclpp::Transport::MRC1));
  EXPECT_TRUE(memory.serialize() == data);
  for (size_t size = 0; size < data.size(); ++size) {
    expectInvalid(
        [&] { mscclpp::RegisteredMemory::deserialize(std::vector<char>(data.begin(), data.begin() + size)); });
  }
  expectInvalid([&] { mscclpp::RegisteredMemory::deserialize(memoryRecord(0)); });
  expectInvalid([&] { mscclpp::RegisteredMemory::deserialize(memoryRecord(2)); });
  mrInfo.addr = 0;
  expectInvalid([&] { mscclpp::RegisteredMemory::deserialize(memoryRecord()); });
}
