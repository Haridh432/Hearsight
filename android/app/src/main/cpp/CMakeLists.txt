cmake_minimum_required(VERSION 3.10.2)

project("hsv2_lite")

# Limit concurrent compilation to 2 jobs to prevent OOM on 8GB RAM systems
set_property(GLOBAL PROPERTY JOB_POOLS compile_jobs=2)
set(CMAKE_JOB_POOL_COMPILE compile_jobs)

# ============================================================
# RealSense Android build configuration
# ============================================================

set(FORCE_RSUSB_BACKEND ON CACHE BOOL "" FORCE)
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_GRAPHICAL_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_WITH_OPENMP OFF CACHE BOOL "" FORCE)
set(BUILD_WITH_TM2 OFF CACHE BOOL "" FORCE)

add_subdirectory(librealsense)

# ============================================================
# Compiler configuration
# ============================================================

# -Werror=format: a printf argument mismatch (e.g. the telemetry JSON) is a crash on the phone.
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fexceptions -Wformat -Werror=format")

include_directories(
        librealsense/include
)

# ============================================================
# Qualcomm AI Runtime (QAIRT / QNN) SDK
#
# Configurable via:
# 1. CMake definition: -DQNN_SDK_ROOT="<path_to_qairt>"
# 2. Environment variables: QNN_SDK_ROOT, QAIRT_SDK, QAIRT_HOME, QNN_HOME
# 3. Default fallback directory: ${CMAKE_CURRENT_SOURCE_DIR}/qairt
# ============================================================

if (NOT DEFINED QNN_SDK_ROOT)
    if (DEFINED ENV{QNN_SDK_ROOT} AND NOT "$ENV{QNN_SDK_ROOT}" STREQUAL "")
        set(QNN_SDK_ROOT "$ENV{QNN_SDK_ROOT}")
    elseif (DEFINED ENV{QAIRT_SDK} AND NOT "$ENV{QAIRT_SDK}" STREQUAL "")
        set(QNN_SDK_ROOT "$ENV{QAIRT_SDK}")
    elseif (DEFINED ENV{QAIRT_HOME} AND NOT "$ENV{QAIRT_HOME}" STREQUAL "")
        set(QNN_SDK_ROOT "$ENV{QAIRT_HOME}")
    elseif (DEFINED ENV{QNN_HOME} AND NOT "$ENV{QNN_HOME}" STREQUAL "")
        set(QNN_SDK_ROOT "$ENV{QNN_HOME}")
    elseif (EXISTS "D:/Qualcomm/qairt/2.47.0.260601")
        set(QNN_SDK_ROOT "D:/Qualcomm/qairt/2.47.0.260601")
    elseif (EXISTS "C:/Qualcomm/qairt/2.47.0.260601")
        set(QNN_SDK_ROOT "C:/Qualcomm/qairt/2.47.0.260601")
    else()
        set(QNN_SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/qairt")
    endif()
endif()

file(TO_CMAKE_PATH "${QNN_SDK_ROOT}" QNN_SDK_ROOT)
set(QNN_INCLUDE_DIR "${QNN_SDK_ROOT}/include")

# Verify standard QAIRT / QNN SDK header layout
set(QNN_HEADERS_FOUND FALSE)
if (EXISTS "${QNN_INCLUDE_DIR}/QNN/QnnInterface.h" OR EXISTS "${QNN_INCLUDE_DIR}/QnnInterface.h")
    set(QNN_HEADERS_FOUND TRUE)
endif()

if (QNN_HEADERS_FOUND)
    message(STATUS "Qualcomm QAIRT/QNN SDK headers located at: ${QNN_SDK_ROOT}")
    include_directories(
            ${QNN_INCLUDE_DIR}
            ${QNN_INCLUDE_DIR}/QNN
            ${QNN_INCLUDE_DIR}/QNN/System
    )
else()
    message(WARNING
        "Qualcomm QAIRT/QNN SDK headers not found at: ${QNN_INCLUDE_DIR}\n"
        "To compile full Hexagon NPU inference, provide Qualcomm QAIRT SDK 2.47+ by:\n"
        "  - Setting environment variable: QNN_SDK_ROOT=<path>\n"
        "  - Passing CMake flag: -DQNN_SDK_ROOT=<path>\n"
        "Required headers: QnnInterface.h, QnnGraph.h, QnnTensor.h, QnnTypes.h, QnnSystemInterface.h")
    include_directories(
            ${QNN_INCLUDE_DIR}
            ${QNN_INCLUDE_DIR}/QNN
            ${QNN_INCLUDE_DIR}/QNN/System
    )
endif()

# ============================================================
# HSV2 Lite native library
# ============================================================

add_library(
        hsv2_lite
        SHARED

        native-lib.cpp
        qnn_runtime.cpp
        yolo_detector.cpp
        imu.cpp
        tracker.cpp
        safety.cpp
        semseg.cpp
        guidance.cpp
        sensor_health.cpp
        safety_validator.cpp
        imu_sync.cpp
        gravity_calib.cpp
        rigid.cpp
        vo.cpp
        recorder.cpp
        local_map.cpp
        nav_plan.cpp
        view3d.cpp
)

# ============================================================
# Android system libraries
# ============================================================

find_library(
        log-lib
        log
)

find_library(
        android-lib
        android
)

# ============================================================
# QNN runtime libraries
#
# These are supplied through android/app/src/main/jniLibs/
# and are loaded dynamically by qnn_runtime.cpp.
#
# We intentionally do NOT link the QNN shared libraries here.
# ============================================================

target_compile_options(
        hsv2_lite
        PRIVATE
        -fexceptions
        -frtti
)

# Per-frame timing logs (E2E / SAFE / VOF, ~90 lines/s) in release builds: off unless
# configured with -DHSV2_TIMING_LOGS=ON (then enable on the phone with
# adb shell setprop debug.hsv2.timing 1).
option(HSV2_TIMING_LOGS "Allow per-frame timing logs in release builds" OFF)
if (HSV2_TIMING_LOGS)
    target_compile_definitions(hsv2_lite PRIVATE HSV2_TIMING_LOGS=1)
endif()

# ============================================================
# Link dependencies
# ============================================================

# ============================================================
# OpenVINS VIO (GPL-3.0; shadow mode, runtime switch debug.hsv2.vio)
# ============================================================
option(HSV2_VIO "Build OpenVINS visual-inertial odometry (GPL-3.0)" OFF)
if (HSV2_VIO)
    set(HSV2_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../../..")
    set(OPENVINS_SRC "/home/naveen/Hearsight/third_party_src/open_vins" CACHE PATH "OpenVINS source tree")
    set(EIGEN3_DIR "/home/naveen/Hearsight/third_party_src/eigen3" CACHE PATH "Eigen 3 headers")
    set(BOOST_HEADERS "/home/naveen/android_libs/boost_headers/include" CACHE PATH "Boost headers")
    set(OpenCV_DIR "/home/naveen/android_libs/OpenCV-android-sdk/sdk/native/jni/abi-${ANDROID_ABI}" CACHE PATH "OpenCV Android SDK")
    include("${HSV2_ROOT}/tools/vio/android/openvins.cmake")
    # Estimator / IMU settings compiled in from tools/vio/config (same files as the PC replay).
    set(VIO_CFG "${HSV2_ROOT}/tools/vio/config")
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
            "${VIO_CFG}/estimator_config.yaml" "${VIO_CFG}/kalibr_imu_chain.yaml")
    file(READ "${VIO_CFG}/estimator_config.yaml" VIO_EST)
    file(READ "${VIO_CFG}/kalibr_imu_chain.yaml" VIO_IMU)
    file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/vio_config_gen.h"
            "#pragma once\n// Generated by CMake from tools/vio/config - do not edit.\n"
            "static const char* kVioEstimatorConfig = R\"HSV2CFG(${VIO_EST})HSV2CFG\";\n"
            "static const char* kVioImuChain = R\"HSV2CFG(${VIO_IMU})HSV2CFG\";\n")
    target_sources(hsv2_lite PRIVATE vio_thread.cpp)
    target_include_directories(hsv2_lite PRIVATE "${CMAKE_CURRENT_BINARY_DIR}")
    target_compile_definitions(hsv2_lite PRIVATE HSV2_VIO=1)
    target_link_libraries(hsv2_lite openvins)
endif()

target_link_libraries(
        hsv2_lite

        realsense2

        ${log-lib}
        ${android-lib}

        dl
)
