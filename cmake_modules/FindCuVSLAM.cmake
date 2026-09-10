# - Find cuVSLAM 15 (https://github.com/nvidia-isaac/cuVSLAM)
#
# An explicit CUVSLAM_ROOT_DIR has priority. Otherwise, the bundled NVIDIA
# binaries are selected from the CUDA major version, Ubuntu release and target
# architecture. Set CUVSLAM_USE_BUNDLED=OFF to search only the host system.
#
# Optional platform overrides are useful when cross-compiling:
#   CUVSLAM_BUNDLED_CUDA_VERSION   12 or 13
#   CUVSLAM_BUNDLED_UBUNTU_VERSION 22, 22.04, 24 or 24.04
#   CUVSLAM_BUNDLED_ARCH           aarch64 or x86_64
#
# Result variables:
#   CUVSLAM_FOUND
#   CUVSLAM_VERSION
#   CUVSLAM_INCLUDE_DIRS
#   CUVSLAM_LIBRARIES
#   CUVSLAM_USING_BUNDLED

find_package(CUDA REQUIRED)
find_package(Eigen3 REQUIRED)

option(CUVSLAM_USE_BUNDLED "Use the bundled cuVSLAM 15 binaries when no explicit root is set" ON)
set(CUVSLAM_ROOT_DIR "" CACHE PATH "Root of an external cuVSLAM installation")
set(CUVSLAM_BUNDLED_CUDA_VERSION "" CACHE STRING "Bundled cuVSLAM CUDA major override (12 or 13)")
set(CUVSLAM_BUNDLED_UBUNTU_VERSION "" CACHE STRING "Bundled cuVSLAM Ubuntu override (22 or 24)")
set(CUVSLAM_BUNDLED_ARCH "" CACHE STRING "Bundled cuVSLAM architecture override")

set(CUVSLAM_USING_BUNDLED FALSE)
set(_CUVSLAM_BUNDLED_ROOT "${CMAKE_CURRENT_LIST_DIR}/../third_party/cuvslam/15.0.0")

set(_CUVSLAM_EXPLICIT_ROOT "${CUVSLAM_ROOT_DIR}")
if(NOT _CUVSLAM_EXPLICIT_ROOT AND DEFINED ENV{CUVSLAM_ROOT_DIR})
    set(_CUVSLAM_EXPLICIT_ROOT "$ENV{CUVSLAM_ROOT_DIR}")
elseif(NOT _CUVSLAM_EXPLICIT_ROOT AND DEFINED ENV{CUVSLAM_ROOT})
    set(_CUVSLAM_EXPLICIT_ROOT "$ENV{CUVSLAM_ROOT}")
endif()

if(_CUVSLAM_EXPLICIT_ROOT)
    find_path(CUVSLAM_INCLUDE_DIR
        NAMES cuvslam/cuvslam2.h cuvslam.h
        HINTS "${_CUVSLAM_EXPLICIT_ROOT}"
        PATH_SUFFIXES include
        NO_DEFAULT_PATH)
    find_library(CUVSLAM_LIBRARY
        NAMES cuvslam
        HINTS "${_CUVSLAM_EXPLICIT_ROOT}"
        PATH_SUFFIXES lib lib64
        NO_DEFAULT_PATH)
elseif(CUVSLAM_USE_BUNDLED AND EXISTS "${_CUVSLAM_BUNDLED_ROOT}")
    set(_CUVSLAM_CUDA_MAJOR "${CUVSLAM_BUNDLED_CUDA_VERSION}")
    if(NOT _CUVSLAM_CUDA_MAJOR)
        set(_CUVSLAM_CUDA_MAJOR "${CUDA_VERSION_MAJOR}")
    endif()
    string(REGEX MATCH "^[0-9]+" _CUVSLAM_CUDA_MAJOR "${_CUVSLAM_CUDA_MAJOR}")

    set(_CUVSLAM_UBUNTU "${CUVSLAM_BUNDLED_UBUNTU_VERSION}")
    if(NOT _CUVSLAM_UBUNTU AND EXISTS "/etc/os-release")
        file(STRINGS "/etc/os-release" _CUVSLAM_OS_VERSION_LINE REGEX "^VERSION_ID=")
        string(REGEX REPLACE "^VERSION_ID=\"?([0-9]+).*" "\\1" _CUVSLAM_UBUNTU "${_CUVSLAM_OS_VERSION_LINE}")
    endif()
    string(REGEX MATCH "^[0-9]+" _CUVSLAM_UBUNTU "${_CUVSLAM_UBUNTU}")

    set(_CUVSLAM_ARCH "${CUVSLAM_BUNDLED_ARCH}")
    if(NOT _CUVSLAM_ARCH)
        string(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" _CUVSLAM_ARCH)
    endif()
    if(_CUVSLAM_ARCH MATCHES "^(arm64|aarch64)$")
        set(_CUVSLAM_ARCH "aarch64")
    elseif(_CUVSLAM_ARCH MATCHES "^(amd64|x86_64)$")
        set(_CUVSLAM_ARCH "x86_64")
    endif()

    set(_CUVSLAM_BUNDLED_LIBRARY
        "${_CUVSLAM_BUNDLED_ROOT}/lib/cu${_CUVSLAM_CUDA_MAJOR}/ubuntu${_CUVSLAM_UBUNTU}/${_CUVSLAM_ARCH}/libcuvslam.so")
    if(EXISTS "${_CUVSLAM_BUNDLED_LIBRARY}")
        set(CUVSLAM_INCLUDE_DIR "${_CUVSLAM_BUNDLED_ROOT}/include")
        set(CUVSLAM_LIBRARY "${_CUVSLAM_BUNDLED_LIBRARY}")
        set(CUVSLAM_VERSION "15.0.0")
        set(CUVSLAM_USING_BUNDLED TRUE)
        message(STATUS
            "Using bundled cuVSLAM 15.0.0: CUDA ${_CUVSLAM_CUDA_MAJOR}, Ubuntu ${_CUVSLAM_UBUNTU}, ${_CUVSLAM_ARCH}")
    else()
        message(STATUS
            "No bundled cuVSLAM binary for CUDA ${_CUVSLAM_CUDA_MAJOR}, Ubuntu ${_CUVSLAM_UBUNTU}, ${_CUVSLAM_ARCH}; searching the system")
    endif()
endif()

if(NOT CUVSLAM_INCLUDE_DIR OR NOT CUVSLAM_LIBRARY)
    find_path(CUVSLAM_INCLUDE_DIR
        NAMES cuvslam/cuvslam2.h cuvslam.h
        PATHS
            /usr/include
            /usr/local/include
            /opt/cuvslam/include
            /opt/ros/humble/share/isaac_ros_nitros/cuvslam/include)
    find_library(CUVSLAM_LIBRARY
        NAMES cuvslam
        PATHS
            /usr/lib
            /usr/local/lib
            /opt/cuvslam/lib
            /opt/ros/humble/share/isaac_ros_nitros/cuvslam/lib)
endif()

if(CUVSLAM_INCLUDE_DIR AND CUVSLAM_LIBRARY)
    if(NOT CUVSLAM_VERSION AND EXISTS "${CUVSLAM_INCLUDE_DIR}/cuvslam.h")
        file(STRINGS "${CUVSLAM_INCLUDE_DIR}/cuvslam.h" _CUVSLAM_VERSION_MAJOR_LINE
            REGEX "^#define CUVSLAM_API_VERSION_MAJOR")
        file(STRINGS "${CUVSLAM_INCLUDE_DIR}/cuvslam.h" _CUVSLAM_VERSION_MINOR_LINE
            REGEX "^#define CUVSLAM_API_VERSION_MINOR")
        string(REGEX MATCH "[0-9]+" _CUVSLAM_VERSION_MAJOR "${_CUVSLAM_VERSION_MAJOR_LINE}")
        string(REGEX MATCH "[0-9]+" _CUVSLAM_VERSION_MINOR "${_CUVSLAM_VERSION_MINOR_LINE}")
        if(_CUVSLAM_VERSION_MAJOR AND _CUVSLAM_VERSION_MINOR)
            set(CUVSLAM_VERSION "${_CUVSLAM_VERSION_MAJOR}.${_CUVSLAM_VERSION_MINOR}.0")
        endif()
    endif()
    if(NOT CUVSLAM_VERSION AND EXISTS "${CUVSLAM_INCLUDE_DIR}/cuvslam/cuvslam2.h")
        set(CUVSLAM_VERSION "15.0.0")
    endif()

    set(CUVSLAM_INCLUDE_DIRS
        "${CUVSLAM_INCLUDE_DIR}"
        ${CUDA_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS})
    set(CUVSLAM_LIBRARIES "${CUVSLAM_LIBRARY}" ${CUDA_LIBRARIES})
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(CuVSLAM
    FOUND_VAR CUVSLAM_FOUND
    REQUIRED_VARS CUVSLAM_LIBRARY CUVSLAM_INCLUDE_DIR
    VERSION_VAR CUVSLAM_VERSION)

if(CUVSLAM_FOUND AND NOT TARGET cuvslam::cuvslam)
    add_library(cuvslam::cuvslam UNKNOWN IMPORTED)
    set_target_properties(cuvslam::cuvslam PROPERTIES
        IMPORTED_LOCATION "${CUVSLAM_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${CUVSLAM_INCLUDE_DIRS}"
        INTERFACE_LINK_LIBRARIES "${CUDA_LIBRARIES};Eigen3::Eigen"
        INTERFACE_COMPILE_FEATURES cxx_std_17)
endif()

mark_as_advanced(CUVSLAM_INCLUDE_DIR CUVSLAM_LIBRARY)
