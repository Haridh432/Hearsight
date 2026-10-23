# Standalone Android build check for OpenVINS (before it goes into the app):
#   cmake -S tools/vio/android -B <dir> -DCMAKE_TOOLCHAIN_FILE=$NDK/build/cmake/android.toolchain.cmake
#         -DANDROID_ABI=arm64-v8a -DANDROID_PLATFORM=android-29 -DCMAKE_BUILD_TYPE=Release
cmake_minimum_required(VERSION 3.18)
project(openvins_android CXX)
set(CMAKE_CXX_STANDARD 17)
set(OPENVINS_SRC "/home/naveen/Hearsight/third_party_src/open_vins" CACHE PATH "")
set(EIGEN3_DIR "/home/naveen/Hearsight/third_party_src/eigen3" CACHE PATH "")
set(BOOST_HEADERS "/home/naveen/android_libs/boost_headers/include" CACHE PATH "")
set(OpenCV_DIR "/home/naveen/android_libs/OpenCV-android-sdk/sdk/native/jni/abi-arm64-v8a" CACHE PATH "")
include(openvins.cmake)

# Link check (unresolved symbols show up here, not in the static library).
add_library(vio_link_check SHARED link_check.cpp)
target_link_libraries(vio_link_check PRIVATE openvins)
target_link_options(vio_link_check PRIVATE -Wl,--no-undefined -Wl,--gc-sections -s)
