# Copyright (c) Microsoft Corporation.
# Licensed under the MIT License.

include(FetchContent)

# Fetch torchcomms headers (header-only dependency — we only need the interface headers)
FetchContent_Declare(torchcomms
    GIT_REPOSITORY https://github.com/meta-pytorch/torchcomms.git
    GIT_TAG v0.2.0-rc2
)
FetchContent_GetProperties(torchcomms)
if(NOT torchcomms_POPULATED)
    FetchContent_Populate(torchcomms)
endif()

# Find PyTorch (provides Torch libraries and Python development headers)
find_package(Torch REQUIRED)
find_package(Python 3.8 COMPONENTS Interpreter Development.Module REQUIRED)

# Locate pybind11 via Python package
execute_process(
    COMMAND "${Python_EXECUTABLE}" -c "import pybind11; print(pybind11.get_cmake_dir())"
    OUTPUT_VARIABLE PYBIND11_CMAKE_DIR
    OUTPUT_STRIP_TRAILING_WHITESPACE
    RESULT_VARIABLE PYBIND11_FIND_RESULT
)
if(PYBIND11_FIND_RESULT EQUAL 0 AND PYBIND11_CMAKE_DIR)
    list(APPEND CMAKE_PREFIX_PATH "${PYBIND11_CMAKE_DIR}")
endif()
find_package(pybind11 REQUIRED)

# Gather our C++ sources. NcclDynamicLoader.cpp is the entry point for the
# separate _comms_nccl target — exclude it from the _comms_mscclpp module.
file(GLOB_RECURSE TORCHCOMM_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/csrc/*.cpp)
list(REMOVE_ITEM TORCHCOMM_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/csrc/NcclDynamicLoader.cpp)

# Torchcomms framework sources we need to compile in directly.
# Our module inherits from TorchWork, TorchCommBackend, and registers with
# TorchCommFactory — these symbols must be in our .so since torchcomms doesn't
# export them from a shared lib we can link against.
set(TORCHCOMMS_FRAMEWORK_SOURCES
    ${torchcomms_SOURCE_DIR}/comms/torchcomms/TorchWork.cpp
    ${torchcomms_SOURCE_DIR}/comms/torchcomms/TorchCommFactory.cpp
    ${torchcomms_SOURCE_DIR}/comms/torchcomms/TorchCommOptions.cpp
    ${torchcomms_SOURCE_DIR}/comms/torchcomms/TorchCommTypes.cpp
    ${torchcomms_SOURCE_DIR}/comms/torchcomms/utils/Utils.cpp
    ${torchcomms_SOURCE_DIR}/comms/torchcomms/utils/StoreManager.cpp
)

# MSCCL++ algorithm selector (same one used by the NCCL extension)
set(MSCCLPP_ALGO_SELECTOR_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/../../src/ext/nccl/algorithm_selector.cc
)

# Build pybind11 module
pybind11_add_module(_comms_mscclpp ${TORCHCOMM_SOURCES} ${TORCHCOMMS_FRAMEWORK_SOURCES} ${MSCCLPP_ALGO_SELECTOR_SOURCES})

# Find glog (required by torchcomms framework sources via Logging.hpp).
# Add the conda/venv prefix to CMAKE_PREFIX_PATH so find_package can locate
# glog's CMake config installed alongside the Python environment.
get_filename_component(_PYTHON_ENV_PREFIX "${Python_EXECUTABLE}" DIRECTORY)
get_filename_component(_PYTHON_ENV_PREFIX "${_PYTHON_ENV_PREFIX}" DIRECTORY)
list(APPEND CMAKE_PREFIX_PATH "${_PYTHON_ENV_PREFIX}")
find_package(glog REQUIRED)

target_include_directories(_comms_mscclpp SYSTEM PRIVATE
    # torchcomms headers: resolves #include <comms/torchcomms/...>
    ${torchcomms_SOURCE_DIR}
    ${GPU_INCLUDE_DIRS}
)
# MSCCL++ internal headers (for algorithm_selector.hpp and debug.h)
target_include_directories(_comms_mscclpp PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../src/ext/nccl
    ${CMAKE_CURRENT_SOURCE_DIR}/../../src/core/include
)

target_link_libraries(_comms_mscclpp PRIVATE
    # MUST use the shared library (not mscclpp_static) to avoid dual-singleton:
    # mscclpp_collectives.so links against libmscclpp.so, so if we statically
    # link mscclpp into our module, there are two copies of singletons like
    # UnixSocketServer::instance(). The bootstrap starts server #1 (static),
    # but the collectives code registers fds in server #2 (shared), causing
    # "Requested fd not found, size of fdSet_ is 0" crashes.
    mscclpp
    mscclpp_collectives
    ${TORCH_LIBRARIES}
    ${GPU_LIBRARIES}
    glog::glog
)

# Propagate USE_ROCM define for mscclpp/gpu.hpp portability
target_compile_definitions(_comms_mscclpp PRIVATE
    $<$<BOOL:${MSCCLPP_USE_ROCM}>:USE_ROCM>
)

target_compile_features(_comms_mscclpp PRIVATE cxx_std_17)

# Set the torch_python library path for linking
set(TORCH_PYTHON_LIB "${TORCH_INSTALL_PREFIX}/lib/libtorch_python.so")
if(EXISTS "${TORCH_PYTHON_LIB}")
    target_link_libraries(_comms_mscclpp PRIVATE "${TORCH_PYTHON_LIB}")
endif()

# Install the module into the package directory for pip/scikit-build-core
install(TARGETS _comms_mscclpp LIBRARY DESTINATION mscclpp_torchcomms COMPONENT torchcomm)

# Copy built module to source tree for easy import during development
add_custom_target(torchcomm_lib_copy ALL
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/_comms_mscclpp*.so
        ${CMAKE_CURRENT_SOURCE_DIR}
    DEPENDS _comms_mscclpp
)

# -----------------------------------------------------------------------------
# Second target: _comms_nccl
#
# This builds the upstream torchcomms NCCL backend sources from
# build-torchcomm/_deps/torchcomms-src/comms/torchcomms/nccl/ into a separate
# .so, paired with our own NcclDynamicLoader.cpp that publishes the
# create_dynamic_loader_nccl entry point torchcomms's TorchCommFactory dlopen
# path requires.
#
# Why we don't reuse the upstream NCCL CMakeLists: it depends on the
# torchcomms top-level CMake project that defines the `torchcomms` static
# library and the ROOT, CONDA_INCLUDE, etc. variables. Mirroring the small
# bit we need here keeps the dependency tree shallow and ensures the .so we
# produce loads correctly through TorchCommFactory.
# -----------------------------------------------------------------------------

set(TORCHCOMMS_NCCL_BACKEND_SOURCES
    ${torchcomms_SOURCE_DIR}/comms/torchcomms/nccl/NcclApi.cpp
    ${torchcomms_SOURCE_DIR}/comms/torchcomms/nccl/TorchCommNCCL.cpp
    ${torchcomms_SOURCE_DIR}/comms/torchcomms/nccl/TorchCommNCCLBootstrap.cpp
    ${torchcomms_SOURCE_DIR}/comms/torchcomms/nccl/TorchCommNCCLCCA.cpp
    ${torchcomms_SOURCE_DIR}/comms/torchcomms/nccl/TorchCommNCCLPy.cpp
    ${torchcomms_SOURCE_DIR}/comms/torchcomms/nccl/TorchCommNCCLUtils.cpp
    ${torchcomms_SOURCE_DIR}/comms/torchcomms/nccl/TorchWorkNCCL.cpp
    ${torchcomms_SOURCE_DIR}/comms/torchcomms/nccl/TorchWorkNCCLQueue.cpp
    ${torchcomms_SOURCE_DIR}/comms/torchcomms/device/cuda/CudaApi.cpp
)

# Framework sources are shared between _comms_mscclpp and _comms_nccl. Each
# .so gets its own copy of TorchCommFactory's singleton (RTLD_LOCAL load) so
# the registrations don't leak across backends.
pybind11_add_module(_comms_nccl
    ${TORCHCOMMS_NCCL_BACKEND_SOURCES}
    ${TORCHCOMMS_FRAMEWORK_SOURCES}
    ${CMAKE_CURRENT_SOURCE_DIR}/csrc/NcclDynamicLoader.cpp
    # TracingGuard.cpp lives outside the framework set used by mscclpp because
    # only the NCCL backend references it (via TracingGuard.hpp).
    ${torchcomms_SOURCE_DIR}/comms/torchcomms/utils/TracingGuard.cpp
)

# Locate libnccl.so.2 shipped with PyTorch (the same one PyTorch's c10d uses).
get_filename_component(_TORCH_LIB_DIR "${TORCH_INSTALL_PREFIX}/lib" ABSOLUTE)
set(_NCCL_PYTORCH_INCLUDE "${TORCH_INSTALL_PREFIX}/../nvidia/nccl/include")
set(_NCCL_PYTORCH_LIB     "${TORCH_INSTALL_PREFIX}/../nvidia/nccl/lib/libnccl.so.2")
if(NOT EXISTS "${_NCCL_PYTORCH_LIB}")
    message(FATAL_ERROR
        "Could not find PyTorch's bundled libnccl.so.2 at ${_NCCL_PYTORCH_LIB}. "
        "Install PyTorch with the nvidia-nccl-cu12 wheel.")
endif()

target_include_directories(_comms_nccl SYSTEM PRIVATE
    ${torchcomms_SOURCE_DIR}
    ${_NCCL_PYTORCH_INCLUDE}
    ${GPU_INCLUDE_DIRS}
)

target_link_libraries(_comms_nccl PRIVATE
    ${TORCH_LIBRARIES}
    ${GPU_LIBRARIES}
    ${_NCCL_PYTORCH_LIB}
    glog::glog
)

if(EXISTS "${TORCH_PYTHON_LIB}")
    target_link_libraries(_comms_nccl PRIVATE "${TORCH_PYTHON_LIB}")
endif()

# Match the NVSHMEM macro used in the historical build (NcclApi.cpp checks it).
target_compile_definitions(_comms_nccl PRIVATE
    USE_NVSHMEM
    # fmt v12 ships in PyTorch's bundled headers (torch/include/fmt/), but only
    # fmt v11 is available as a linkable shared object (libfmt.so.11 in conda).
    # Compiling against v12 headers and linking against v11 yields an
    # ``undefined symbol: fmt::v12::vformat...`` at dlopen time. Force the
    # header-only build of whichever fmt headers are picked so all fmt code
    # is inlined into _comms_nccl.so itself, eliminating the runtime dep.
    FMT_HEADER_ONLY
)

target_compile_features(_comms_nccl PRIVATE cxx_std_20)

install(TARGETS _comms_nccl LIBRARY DESTINATION mscclpp_torchcomms COMPONENT torchcomm)

add_custom_target(torchcomm_nccl_copy ALL
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/_comms_nccl*.so
        ${CMAKE_CURRENT_SOURCE_DIR}
    DEPENDS _comms_nccl
)
