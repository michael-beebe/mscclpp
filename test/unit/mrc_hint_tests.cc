// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#include <array>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <mscclpp/env.hpp>

#include "../framework.hpp"
#include "ib.hpp"
#include "mrc_hints.hpp"

namespace {

template <typename Function>
bool invalidUsage(Function&& function) {
  try {
    function();
  } catch (const mscclpp::Error& error) {
    return error.getErrorCode() == mscclpp::ErrorCode::InvalidUsage;
  }
  return false;
}

// CPU-only resource double; never calls a verbs/MRC library or a device.
struct HintBackend {
  struct Hint {
    int refs = 0;
  };
  struct Qp {
    Hint* hint = nullptr;
  };
  static inline bool failQp = false;
  static inline bool failHint = false;
  static inline std::string events;
  static int destroyQp(Qp* qp) {
    events += 'Q';
    if (failQp) return 5;
    if (qp->hint) --qp->hint->refs;
    return 0;
  }
  static int destroyHint(Hint* hint) {
    EXPECT_EQ(hint->refs, 0);
    events += 'H';
    return failHint ? 5 : 0;
  }
};

}  // namespace

TEST(MrcHintTest, IntegerParsingIsStrict) {
  using mscclpp::detail::parseMrcHintInteger;
  EXPECT_EQ(parseMrcHintInteger("rate", "1048576"), int64_t{1048576});
  EXPECT_EQ(parseMrcHintInteger("rate", "4294967295"), int64_t{4294967295});
  EXPECT_EQ(parseMrcHintInteger("rate", "-1"), int64_t{-1});
  for (auto text : {"", "12junk", "garbage", "1.5", " 1", "9223372036854775808"}) {
    EXPECT_TRUE(invalidUsage([&] { parseMrcHintInteger("rate", text); }));
  }
}

TEST(MrcHintTest, EnableRequiresZeroOrOne) {
  EXPECT_FALSE(mscclpp::detail::parseMrcHintEnable("0"));
  EXPECT_TRUE(mscclpp::detail::parseMrcHintEnable("1"));
  for (auto text : {"", "2", "-1", "true", "1junk"}) {
    EXPECT_TRUE(invalidUsage([&] { mscclpp::detail::parseMrcHintEnable(text); }));
  }
}

TEST(MrcHintTest, ZeroRatesPreserveProviderDefaults) {
  std::array<unsigned char, 16> zero{};
  for (int qps : {1, 2, 3, 8}) {
    auto hint = mscclpp::detail::makeMrcCcHint(0, 0, 0, qps);
    EXPECT_EQ(std::memcmp(&hint, zero.data(), zero.size()), 0);
  }
  auto truncated = mscclpp::detail::makeMrcCcHint(0, 1, 0, 8);
  EXPECT_EQ(std::memcmp(&truncated, zero.data(), zero.size()), 0);
}

TEST(MrcHintTest, VersionOneAbiAndMinimumDivision) {
  static_assert(sizeof(mscclpp::detail::MrcCcHint) == 16);
  constexpr int64_t max = std::numeric_limits<uint32_t>::max();
  for (int qps : {1, 2, 3, 8}) {
    for (auto rates : {std::array<int64_t, 3>{1048576, 65536, 1048576},
                       {100, 0, 0},
                       {0, 65536, 0},
                       {0, 0, 1048576},
                       {max, max * qps, max}}) {
      auto hint = mscclpp::detail::makeMrcCcHint(rates[0], rates[1], rates[2], qps);
      std::array<unsigned char, 16> expected{};
      expected[0] = 1;
      std::array<uint32_t, 3> encoded{static_cast<uint32_t>(rates[0]), static_cast<uint32_t>(rates[1] / qps),
                                      static_cast<uint32_t>(rates[2])};
      std::memcpy(expected.data() + 4, encoded.data(), sizeof(encoded));
      EXPECT_EQ(std::memcmp(&hint, expected.data(), expected.size()), 0);
    }
  }
}

TEST(MrcHintTest, InvalidRatesAreRejected) {
  constexpr int64_t overflow = int64_t{std::numeric_limits<uint32_t>::max()} + 1;
  for (auto bad : {std::array<int64_t, 3>{-1, 0, 0},
                   {0, -1, 0},
                   {0, 0, -1},
                   {overflow, 0, 0},
                   {0, overflow, 0},
                   {0, 0, overflow}}) {
    EXPECT_TRUE(invalidUsage([&] { mscclpp::detail::makeMrcCcHint(bad[0], bad[1], bad[2]); }));
  }
  EXPECT_TRUE(invalidUsage([] { mscclpp::detail::makeMrcCcHint(0, 0, 0, 0); }));
  EXPECT_TRUE(invalidUsage([] { mscclpp::detail::makeMrcCcHint(0, 0, 0, -1); }));
}

TEST(MrcHintTest, CleanupOrdersQpBeforeHint) {
  HintBackend::Hint storage{1};
  HintBackend::Qp qpStorage{&storage};
  auto* qp = &qpStorage;
  auto* hint = &storage;
  HintBackend::events.clear();
  EXPECT_TRUE(mscclpp::detail::destroyQpAndHint<HintBackend>(qp, hint));
  EXPECT_TRUE(HintBackend::events == "QH");
  EXPECT_EQ(qp, nullptr);
  EXPECT_EQ(hint, nullptr);
  EXPECT_TRUE(mscclpp::detail::destroyQpAndHint<HintBackend>(qp, hint));
  EXPECT_TRUE(HintBackend::events == "QH");

  hint = &storage;  // QP construction failed after the hint was created.
  HintBackend::events.clear();
  EXPECT_TRUE(mscclpp::detail::destroyQpAndHint<HintBackend>(qp, hint));
  EXPECT_TRUE(HintBackend::events == "H");
  qp = &qpStorage;
  qpStorage.hint = nullptr;  // Hints disabled.
  HintBackend::events.clear();
  EXPECT_TRUE(mscclpp::detail::destroyQpAndHint<HintBackend>(qp, hint));
  EXPECT_TRUE(HintBackend::events == "Q");
}

TEST(MrcHintTest, CleanupFailuresRetainDependentHandles) {
  HintBackend::Hint storage{1};
  HintBackend::Qp qpStorage{&storage};
  auto* qp = &qpStorage;
  auto* hint = &storage;
  HintBackend::events.clear();
  HintBackend::failQp = true;
  bool first = mscclpp::detail::destroyQpAndHint<HintBackend>(qp, hint);
  HintBackend::failQp = false;
  EXPECT_FALSE(first);
  EXPECT_EQ(qp, &qpStorage);
  EXPECT_EQ(hint, &storage);
  EXPECT_TRUE(HintBackend::events == "Q");
  EXPECT_EQ(storage.refs, 1);
  HintBackend::failHint = true;
  bool second = mscclpp::detail::destroyQpAndHint<HintBackend>(qp, hint);
  HintBackend::failHint = false;
  EXPECT_FALSE(second);
  EXPECT_EQ(qp, nullptr);
  EXPECT_EQ(hint, &storage);
  EXPECT_TRUE(HintBackend::events == "QQH");
  EXPECT_TRUE(mscclpp::detail::destroyQpAndHint<HintBackend>(qp, hint));
  EXPECT_EQ(hint, nullptr);
  EXPECT_TRUE(HintBackend::events == "QQHH");
}

TEST(MrcHintTest, EnvironmentDefaults) {
  if (std::getenv("MSCCLPP_MRC_QP_HINT_ENABLE")) {
    SKIP_TEST() << "Run with hint options unset for defaults or use the configured-options test";
  }
  auto config = mscclpp::env();
  EXPECT_FALSE(config->mrcQpHintEnable);
  EXPECT_EQ(config->mrcCcInitRate, int64_t{0});
  EXPECT_EQ(config->mrcCcMinRate, int64_t{0});
  EXPECT_EQ(config->mrcCcMaxRate, int64_t{0});
}

TEST(MrcHintTest, EnvironmentReadsConfiguredOptions) {
  if (!std::getenv("MSCCLPP_TEST_MRC_HINT_ENV")) SKIP_TEST() << "Requires the configured environment fixture";
  auto config = mscclpp::env();
  EXPECT_TRUE(config->mrcQpHintEnable);
  EXPECT_EQ(config->mrcCcInitRate, int64_t{1048576});
  EXPECT_EQ(config->mrcCcMinRate, int64_t{65536});
  EXPECT_EQ(config->mrcCcMaxRate, int64_t{1048576});
}

TEST(MrcHintTest, EnvironmentDisabledIgnoresRates) {
  if (!std::getenv("MSCCLPP_TEST_MRC_HINT_DISABLED")) SKIP_TEST() << "Requires the disabled-hints fixture";
  auto config = mscclpp::env();
  EXPECT_FALSE(config->mrcQpHintEnable);
  EXPECT_EQ(config->mrcCcInitRate, int64_t{0});
  EXPECT_EQ(config->mrcCcMinRate, int64_t{0});
  EXPECT_EQ(config->mrcCcMaxRate, int64_t{0});
}

TEST(MrcHintTest, EnvironmentRejectsMalformedOptions) {
  const char* variable = std::getenv("MSCCLPP_TEST_MRC_HINT_BAD_ENV");
  if (!variable) SKIP_TEST() << "Requires the malformed environment fixture";
  try {
    mscclpp::env();
    FAIL() << "Expected the malformed option to fail without recursive logging";
  } catch (const mscclpp::Error& error) {
    EXPECT_TRUE(error.getErrorCode() == mscclpp::ErrorCode::InvalidUsage);
    EXPECT_TRUE(std::string(error.what()).find(variable) != std::string::npos);
  }
}

TEST(MrcHintTest, NativeConfiguredResourceLifetime) {
#if defined(MSCCLPP_USE_MRC)
  if (!std::getenv("MSCCLPP_TEST_NATIVE_MRC")) SKIP_TEST() << "Requires explicit native MRC resource-test opt-in";
  for (int i = 0; i < 3; ++i) {
    auto context = mscclpp::Context::create();
    mscclpp::EndpointConfig config(mscclpp::Transport::MRC0, mscclpp::DeviceType::CPU);
    config.ib.mode = mscclpp::EndpointConfig::Ib::Mode::HostNoAtomic;
    config.ib.gidIndex = 3;
    config.ib.maxCqSize = config.ib.maxCqPollNum = config.ib.maxRecvWr = 8;
    config.ib.maxSendWr = 16;
    auto endpoint = context->createEndpoint(config);
    auto bytes = endpoint.serialize();
    std::weak_ptr<mscclpp::Context> owner = context;
    context.reset();
    EXPECT_TRUE(owner.expired());
    EXPECT_TRUE(endpoint.serialize() == bytes);
    // The external test auditor verifies hint creation/INIT attachment and that
    // endpoint destruction releases QP -> hint -> CQs -> context without leaks.
  }
#else
  SKIP_TEST() << "Native MRC support is not enabled";
#endif
}

TEST(MrcHintTest, NativeHintFailurePropagates) {
#if defined(MSCCLPP_USE_MRC)
  const char* stage = std::getenv("MSCCLPP_TEST_MRC_HINT_FAILURE");
  if (!stage) SKIP_TEST() << "Requires an external native API failure-injection fixture";
  EXPECT_TRUE(mscclpp::env()->mrcQpHintEnable);
  auto context = mscclpp::Context::create();
  mscclpp::EndpointConfig config(mscclpp::Transport::MRC0, mscclpp::DeviceType::CPU);
  config.ib.mode = mscclpp::EndpointConfig::Ib::Mode::HostNoAtomic;
  config.ib.gidIndex = 3;
  try {
    context->createEndpoint(config);
    FAIL() << "Requested MRC hint failures must not fall back or be ignored";
  } catch (const mscclpp::BaseError& error) {
    const char* expected = std::strcmp(stage, "hint") == 0 ? "mrc_create_qp_hint failed"
                           : std::strcmp(stage, "qp") == 0 ? "MRC create QP failed"
                                                           : "MRC modify QP failed";
    EXPECT_TRUE(std::string(error.what()).find(expected) != std::string::npos);
  }
#else
  SKIP_TEST() << "Native MRC support is not enabled";
#endif
}