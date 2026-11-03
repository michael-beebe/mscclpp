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

MrcCtx::MrcCtx(const std::string& devName)
    : devName_(devName),
      context_(nullptr, ibv_close_device),
      pd_(nullptr, ibv_dealloc_pd),
      mrcContext_(nullptr, mrc_destroy_context) {
  if (devName.empty()) {
    throw Error("Native MRC requires a device name", ErrorCode::InvalidUsage);
  }

  int numDevices = 0;
  std::unique_ptr<ibv_device*, decltype(&ibv_free_device_list)> devices(ibv_get_device_list(&numDevices),
                                                                        ibv_free_device_list);
  if (!devices) {
    throw IbError("ibv_get_device_list failed", errno);
  }
  for (int i = 0; i < numDevices; ++i) {
    if (devName == ibv_get_device_name(devices.get()[i])) {
      context_.reset(ibv_open_device(devices.get()[i]));
      if (!context_) {
        throw IbError("ibv_open_device failed for " + devName, errno);
      }
      break;
    }
  }
  if (!context_) {
    throw Error("Native MRC device not found: " + devName, ErrorCode::InvalidUsage);
  }

  // TODO: Use mrc_query_device_ex and its attribute layout when adding newer SDK support.
  mrc_attr capabilities{};
  int supported = 0;
  if (mrc_query_device(context_.get(), &capabilities, &supported) != 0) {
    throw IbError("mrc_query_device failed for " + devName, errno);
  }
  if (!supported) {
    throw Error("Device does not support native MRC: " + devName, ErrorCode::InvalidUsage);
  }

  pd_.reset(ibv_alloc_pd(context_.get()));
  if (!pd_) {
    throw IbError("ibv_alloc_pd failed for " + devName, errno);
  }

  // TODO: Add runtime compatibility checks with mrc_query_version when the SDK provides it.
  mrc_context_attr attrs{};
  attrs.mrc_api_version_used = MRC_API_VER(1, 0, 0);
  mrcContext_.reset(mrc_create_context(context_.get(), &attrs));
  if (!mrcContext_) {
    throw IbError("mrc_create_context failed for " + devName, errno);
  }
}

}  // namespace mscclpp

#endif  // defined(MSCCLPP_USE_MRC)
