# - Find cuVSLAM library (https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam)
#
#  CUVSLAM_ROOT / CUVSLAM_ROOT_DIR environment variables can point to either
#  the cuVSLAM source directory (containing libs/ and build/) or the build
#  directory directly.
#
# It sets the following variables:
#  CUVSLAM_FOUND         - Set to false, or undefined, if cuVSLAM isn't found.
#  CUVSLAM_VERSION       - The version of cuVSLAM found (e.g., "15.0.0").
#  CUVSLAM_INCLUDE_DIRS  - The cuVSLAM include directories.
#  CUVSLAM_LIBRARIES     - The cuVSLAM library to link against.

find_package(CUDA REQUIRED)
find_package(Eigen3 REQUIRED)

# cuVSLAM v15 headers live under a cuvslam/ subdirectory.
# find_path returns the parent directory (the include root).
find_path(CUVSLAM_INCLUDE_DIR
    NAMES cuvslam/cuvslam2.h
    HINTS
        $ENV{CUVSLAM_ROOT}/include
        $ENV{CUVSLAM_ROOT}/libs          # source-tree case: libs/cuvslam/cuvslam2.h
        $ENV{CUVSLAM_ROOT_DIR}/include
        $ENV{CUVSLAM_ROOT_DIR}/libs
    PATHS
        /opt/cuvslam/include
        /usr/local/include
        /usr/include
)

find_library(CUVSLAM_LIBRARY
    NAMES cuvslam
    HINTS
        $ENV{CUVSLAM_ROOT}/lib
        $ENV{CUVSLAM_ROOT}/bin           # cuVSLAM build output goes to bin/
        $ENV{CUVSLAM_ROOT}/build/bin     # source-tree case: build/bin/libcuvslam.so
        $ENV{CUVSLAM_ROOT_DIR}/lib
        $ENV{CUVSLAM_ROOT_DIR}/bin
        $ENV{CUVSLAM_ROOT_DIR}/build/bin
    PATHS
        /opt/cuvslam/lib
        /usr/local/lib
        /usr/lib
)

# Locate version.h – it is generated during the build so it may be in a
# separate generated/ directory rather than alongside the other headers.
set(CUVSLAM_VERSION_H_PATH "")
if(CUVSLAM_INCLUDE_DIR)
    foreach(_vh
            "${CUVSLAM_INCLUDE_DIR}/cuvslam/version.h"
            "${CUVSLAM_INCLUDE_DIR}/version.h")
        if(EXISTS "${_vh}")
            set(CUVSLAM_VERSION_H_PATH "${_vh}")
            break()
        endif()
    endforeach()
endif()

if(NOT CUVSLAM_VERSION_H_PATH AND CUVSLAM_LIBRARY)
    get_filename_component(_lib_dir "${CUVSLAM_LIBRARY}" DIRECTORY)
    foreach(_vh
            "${_lib_dir}/../generated/version.h"
            "${_lib_dir}/../../generated/version.h")
        get_filename_component(_vh_abs "${_vh}" ABSOLUTE)
        if(EXISTS "${_vh_abs}")
            set(CUVSLAM_VERSION_H_PATH "${_vh_abs}")
            break()
        endif()
    endforeach()
endif()

if(CUVSLAM_INCLUDE_DIR AND CUVSLAM_LIBRARY)
    # Extract version from version.h (CUVSLAM_API_VERSION_MAJOR / _MINOR defines)
    if(CUVSLAM_VERSION_H_PATH)
        file(STRINGS "${CUVSLAM_VERSION_H_PATH}" CUVSLAM_VERSION_MAJOR_LINE
             REGEX "^#define CUVSLAM_API_VERSION_MAJOR")
        file(STRINGS "${CUVSLAM_VERSION_H_PATH}" CUVSLAM_VERSION_MINOR_LINE
             REGEX "^#define CUVSLAM_API_VERSION_MINOR")
        if(CUVSLAM_VERSION_MAJOR_LINE AND CUVSLAM_VERSION_MINOR_LINE)
            string(REGEX MATCH "[0-9]+" CUVSLAM_VERSION_MAJOR "${CUVSLAM_VERSION_MAJOR_LINE}")
            string(REGEX MATCH "[0-9]+" CUVSLAM_VERSION_MINOR "${CUVSLAM_VERSION_MINOR_LINE}")
            set(CUVSLAM_VERSION "${CUVSLAM_VERSION_MAJOR}.${CUVSLAM_VERSION_MINOR}.0")
        endif()
    endif()

    set(CUVSLAM_INCLUDE_DIRS ${CUVSLAM_INCLUDE_DIR})

    # Add the generated directory (for version.h) when it is separate from the main include dir
    if(CUVSLAM_VERSION_H_PATH)
        get_filename_component(_gen_dir "${CUVSLAM_VERSION_H_PATH}" DIRECTORY)
        if(NOT _gen_dir STREQUAL "${CUVSLAM_INCLUDE_DIR}"
           AND NOT _gen_dir STREQUAL "${CUVSLAM_INCLUDE_DIR}/cuvslam")
            list(APPEND CUVSLAM_INCLUDE_DIRS "${_gen_dir}")
        endif()
    endif()

    list(APPEND CUVSLAM_INCLUDE_DIRS ${CUDA_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

    set(CUVSLAM_LIBRARIES
        ${CUVSLAM_LIBRARY}
        ${CUDA_LIBRARIES}
    )
endif()

# Version compatibility check - cuVSLAM only guarantees API compatibility within the same major version
set(CUVSLAM_VERSION_MISMATCH_REASON "")
if(CuVSLAM_FIND_VERSION AND CUVSLAM_VERSION)
    string(REGEX MATCH "^[0-9]+" REQUESTED_MAJOR_VERSION "${CuVSLAM_FIND_VERSION}")
    if(NOT CUVSLAM_VERSION_MAJOR EQUAL REQUESTED_MAJOR_VERSION)
        set(CUVSLAM_VERSION_MISMATCH_REASON "Major version mismatch: found ${CUVSLAM_VERSION_MAJOR}.x but requested ${REQUESTED_MAJOR_VERSION}.x.\ncuVSLAM only guarantees API compatibility within the same major version.\nPlease install cuVSLAM ${REQUESTED_MAJOR_VERSION}.x or update CMakeLists.txt to request version ${CUVSLAM_VERSION_MAJOR}.0.0")
        
        if(CuVSLAM_FIND_REQUIRED)
            message(FATAL_ERROR 
                "cuVSLAM major version mismatch: found version ${CUVSLAM_VERSION} but version ${CuVSLAM_FIND_VERSION} is required.\n"
                "cuVSLAM only guarantees API compatibility within the same major version.\n"
                "Found major version ${CUVSLAM_VERSION_MAJOR} is not compatible with requested major version ${REQUESTED_MAJOR_VERSION}.\n"
                "Please install cuVSLAM ${REQUESTED_MAJOR_VERSION}.x or update CMakeLists.txt to request version ${CUVSLAM_VERSION_MAJOR}.x."
            )
        else()
            # Clear the found variables to indicate incompatibility
            unset(CUVSLAM_LIBRARIES)
            unset(CUVSLAM_INCLUDE_DIRS)
        endif()
    endif()
endif()

# Handle the QUIET and REQUIRED arguments
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(CuVSLAM
    FOUND_VAR CUVSLAM_FOUND
    REQUIRED_VARS CUVSLAM_LIBRARIES CUVSLAM_INCLUDE_DIRS
    VERSION_VAR CUVSLAM_VERSION
    REASON_FAILURE_MESSAGE "${CUVSLAM_VERSION_MISMATCH_REASON}"
    HANDLE_COMPONENTS
)

if(CUVSLAM_FOUND)
    # Create imported target for modern CMake usage
    if(NOT TARGET cuvslam::cuvslam)
        add_library(cuvslam::cuvslam UNKNOWN IMPORTED)
        set_target_properties(cuvslam::cuvslam PROPERTIES
            IMPORTED_LOCATION "${CUVSLAM_LIBRARY}"
            INTERFACE_INCLUDE_DIRECTORIES "${CUVSLAM_INCLUDE_DIRS}"
            INTERFACE_LINK_LIBRARIES "${CUVSLAM_LIBRARIES};Eigen3::Eigen"
        )
    endif()
endif()

mark_as_advanced(CUVSLAM_INCLUDE_DIR CUVSLAM_LIBRARY)
