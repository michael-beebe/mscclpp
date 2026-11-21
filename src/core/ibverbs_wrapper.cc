// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#if defined(USE_IBVERBS)

#include "ibverbs_wrapper.hpp"

#include <dlfcn.h>

#include <cstdlib>
#include <memory>
#include <mscclpp/env.hpp>

#include "logger.hpp"

// Adding MSCCLPP_USE_MRC micro for MRC enablement.
// Non-MRC environments will not be affected by this macro as long as VMRC_LIBIBVERBS_SO
// environment variable is not set.
#if (MSCCLPP_USE_MRC)
#include <set>
#endif  // (MSCCLPP_USE_MRC)

namespace mscclpp {

static std::unique_ptr<void, int (*)(void*)> globalIBVerbsHandle(nullptr, &::dlclose);
static std::unique_ptr<void, int (*)(void*)> globalOrigIBVerbsHandle(nullptr, &::dlclose);

// Native MRC shares PD/MR APIs with IB, but must bypass MSCCLPP_IBV_SO for
// device/context lifecycle too. The original-library override also preserves
// the existing IB-over-shim mode, independently of the native MRC provider.
static void* originalIBVerbsHandle() {
  if (!globalOrigIBVerbsHandle) {
    const char* override = ::getenv("VMRC_LIBIBVERBS_SO");
    const char* names[] = {override ? override : "libibverbs.so.1", "libibverbs.so", nullptr};
    for (int i = 0; names[i]; ++i) {
      void* handle = ::dlopen(names[i], RTLD_NOW);
      if (handle) {
        globalOrigIBVerbsHandle.reset(handle);
        break;
      }
      if (override) break;  // Never silently replace an explicit library.
    }
    if (!globalOrigIBVerbsHandle) {
      THROW(NET, SysError, errno, "Failed to load native libibverbs: ", std::string(::dlerror()));
    }
  }
  return globalOrigIBVerbsHandle.get();
}

template <bool Native>
void* VerbsLib<Native>::dlsym(const std::string& symbol, bool allowReturnNull) {
  if constexpr (Native) {
    void* ptr = ::dlsym(originalIBVerbsHandle(), symbol.c_str());
    if (!ptr && !allowReturnNull) {
      THROW(NET, SysError, errno, "Failed to load native libibverbs symbol: ", symbol);
    }
    return ptr;
  }
#if (MSCCLPP_USE_MRC)
  static std::set<std::string> mrcSymbols = {
      "ibv_get_device_list", "ibv_get_device_name", "ibv_open_device", "ibv_close_device", "ibv_query_qp",
      "ibv_create_cq",       "ibv_destroy_cq",      "ibv_create_qp",   "ibv_modify_qp",    "ibv_destroy_qp",
  };
#endif  // (MSCCLPP_USE_MRC)
  if (!globalIBVerbsHandle) {
    if (mscclpp::env()->ibvSo != "") {
      void* handle = ::dlopen(mscclpp::env()->ibvSo.c_str(), RTLD_NOW);
      if (handle) {
        globalIBVerbsHandle.reset(handle);
      } else {
        THROW(NET, SysError, errno, "Failed to load libibverbs library specified by MSCCLPP_IBV_SO ('",
              mscclpp::env()->ibvSo, "'): ", std::string(::dlerror()));
      }
    } else {
      const char* possibleLibNames[] = {"libibverbs.so", "libibverbs.so.1", nullptr};
      for (int i = 0; possibleLibNames[i] != nullptr; i++) {
        void* handle = ::dlopen(possibleLibNames[i], RTLD_NOW);
        if (handle) {
          globalIBVerbsHandle.reset(handle);
          break;
        }
      }
    }
    if (!globalIBVerbsHandle) {
      THROW(NET, SysError, errno, "Failed to open libibverbs: ", std::string(::dlerror()));
    }
  }
#if (MSCCLPP_USE_MRC)
  // In MRC mode, `VMRC_LIBIBVERBS_SO` should be set.
  char* vmrcLibibverbsSo = ::getenv("VMRC_LIBIBVERBS_SO");
  void* ptr;
  if (vmrcLibibverbsSo != nullptr && mrcSymbols.find(symbol) == mrcSymbols.end()) {
    // If we are in MRC mode and the symbol is not in the table, get it from the original libibverbs.
    ptr = ::dlsym(originalIBVerbsHandle(), symbol.c_str());
  } else {
    ptr = ::dlsym(globalIBVerbsHandle.get(), symbol.c_str());
  }
#else   // !(MSCCLPP_USE_MRC)
  void* ptr = ::dlsym(globalIBVerbsHandle.get(), symbol.c_str());
#endif  // !(MSCCLPP_USE_MRC)
  if (!ptr && !allowReturnNull) {
    THROW(NET, SysError, errno, "Failed to load libibverbs symbol: ", symbol);
  }
  return ptr;
}

template <bool Native>
bool VerbsLib<Native>::isDmabufSupported() {
  static int isSupported = -1;
  if (isSupported == -1) {
    void* ptr = VerbsLib::dlsym("ibv_reg_dmabuf_mr", true);
    isSupported = (ptr != nullptr);
    if (!isSupported) {
      DEBUG(NET, "This platform does not support DMABUF");
    }
  }
  return isSupported;
}

template <bool Native>
struct ibv_mr* VerbsLib<Native>::ibv_reg_dmabuf_mr(struct ibv_pd* pd, uint64_t offset, size_t length, uint64_t iova,
                                                   int fd, int access) {
  using FuncType = struct ibv_mr* (*)(struct ibv_pd*, uint64_t, size_t, uint64_t, int, int);
  static FuncType impl = nullptr;
  if (!isDmabufSupported())
    THROW(NET, Error, ErrorCode::InvalidUsage, "libibverbs does not support ibv_reg_dmabuf_mr in this platform.");
  if (!impl) impl = reinterpret_cast<FuncType>(VerbsLib::dlsym("ibv_reg_dmabuf_mr"));
  return impl(pd, offset, length, iova, fd, access);
}

template struct VerbsLib<false>;
template struct VerbsLib<true>;

}  // namespace mscclpp

#endif  // defined(USE_IBVERBS)
