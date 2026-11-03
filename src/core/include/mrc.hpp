// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#ifndef MSCCLPP_MRC_HPP_
#define MSCCLPP_MRC_HPP_

#if defined(MSCCLPP_USE_MRC)

#include <memory>
#include <string>

struct ibv_context;
struct ibv_pd;
struct mrc_context;

namespace mscclpp {

class MrcCtx {
 public:
  explicit MrcCtx(const std::string& devName);
  MrcCtx(const MrcCtx&) = delete;
  MrcCtx& operator=(const MrcCtx&) = delete;

  const std::string& getDevName() const { return devName_; }

 private:
  const std::string devName_;
  std::unique_ptr<ibv_context, int (*)(ibv_context*)> context_;
  std::unique_ptr<ibv_pd, int (*)(ibv_pd*)> pd_;
  std::unique_ptr<mrc_context, int (*)(mrc_context*)> mrcContext_;
};

}  // namespace mscclpp

#endif  // defined(MSCCLPP_USE_MRC)
#endif  // MSCCLPP_MRC_HPP_
