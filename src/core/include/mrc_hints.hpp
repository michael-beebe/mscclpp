// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#ifndef MSCCLPP_MRC_HINTS_HPP_
#define MSCCLPP_MRC_HINTS_HPP_

#include <charconv>
#include <cstdint>
#include <limits>
#include <mscclpp/errors.hpp>
#include <string>
#include <string_view>

namespace mscclpp::detail {

// Parse without logging: Env construction must not recursively call env().
inline int64_t parseMrcHintInteger(std::string_view name, std::string_view value) {
  if (value.empty()) {
    throw Error(std::string(name) + " must not be empty", ErrorCode::InvalidUsage);
  }
  int64_t result = 0;
  auto parsed = std::from_chars(value.data(), value.data() + value.size(), result);
  if (parsed.ec != std::errc{} || parsed.ptr != value.data() + value.size()) {
    throw Error(std::string(name) + " must be a decimal integer fitting int64_t", ErrorCode::InvalidUsage);
  }
  return result;
}

inline bool parseMrcHintEnable(std::string_view value) {
  auto enabled = parseMrcHintInteger("MSCCLPP_MRC_QP_HINT_ENABLE", value);
  if (enabled != 0 && enabled != 1) {
    throw Error("MSCCLPP_MRC_QP_HINT_ENABLE must be 0 or 1", ErrorCode::InvalidUsage);
  }
  return enabled != 0;
}

// The native plugin's version-1 vendor ABI. Reserved bytes and the rest of the
// SDK's vendor_cfg array remain zero; rates use provider units, not bits/s.
struct MrcCcHint {
  uint8_t version;
  uint8_t reserved[3];
  uint32_t initRate;
  uint32_t minRate;
  uint32_t maxRate;
};
static_assert(sizeof(MrcCcHint) == 16);

inline MrcCcHint makeMrcCcHint(int64_t initRate, int64_t minRate, int64_t maxRate, int numQpsPerPeer = 1) {
  if (numQpsPerPeer <= 0) {
    throw Error("MRC QP hint requires a positive QP count", ErrorCode::InvalidUsage);
  }
  constexpr auto max = std::numeric_limits<uint32_t>::max();
  if (initRate < 0 || minRate < 0 || maxRate < 0 || initRate > max || maxRate > max || minRate / numQpsPerPeer > max) {
    throw Error("MRC CC rates must be nonnegative and fit uint32_t per QP", ErrorCode::InvalidUsage);
  }
  minRate /= numQpsPerPeer;
  MrcCcHint hint{};
  if (initRate || minRate || maxRate) {
    hint.version = 1;
    hint.initRate = static_cast<uint32_t>(initRate);
    hint.minRate = static_cast<uint32_t>(minRate);
    hint.maxRate = static_cast<uint32_t>(maxRate);
  }
  return hint;
}

// Keep a failed-to-destroy QP's attached hint intact. Likewise, do not lose the
// hint handle on a failed hint destruction. Callers can report/retry cleanup.
template <typename Backend>
bool destroyQpAndHint(typename Backend::Qp*& qp, typename Backend::Hint*& hint) {
  if (qp) {
    if (Backend::destroyQp(qp) != 0) return false;
    qp = nullptr;
  }
  if (hint) {
    if (Backend::destroyHint(hint) != 0) return false;
    hint = nullptr;
  }
  return true;
}

}  // namespace mscclpp::detail

#endif  // MSCCLPP_MRC_HINTS_HPP_