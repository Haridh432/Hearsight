# PC replay of HearSight recordings (format v4) through OpenVINS.
#
#   cmake -S tools/vio -B <build> -DOPENVINS_SRC=<open_vins> -DOPENVINS_BUILD=<ov_msckf build>
#   cmake --build <build> -j
#
# OpenVINS (GPL-3.0) is built separately without ROS:
#   cmake <open_vins>/ov_msckf -DENABLE_ROS=OFF -DCMAKE_BUILD_TYPE=Release
#         -DCMAKE_DISABLE_FIND_PACKAGE_ament_cmake=ON -DCMAKE_DISABLE_FIND_PACKAGE_catkin=ON
# (ov_core and ov_init are compiled into libov_msckf_lib.so in that mode.)
cmake_minimum_required(VERSION 3.10)
project(hearsight_replay_vio CXX)
set(CMAKE_CXX_STANDARD 17)
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif ()

set(OPENVINS_SRC "/home/naveen/Hearsight/third_party_src/open_vins" CACHE PATH "OpenVINS source tree")
set(OPENVINS_BUILD "/home/naveen/Hearsight/third_party_src/open_vins_build" CACHE PATH "OpenVINS ov_msckf build dir")
set(HSV2_CPP "${CMAKE_CURRENT_SOURCE_DIR}/../../android/app/src/main/cpp")

find_package(Eigen3 REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem thread date_time)
find_package(Ceres REQUIRED)
find_package(Threads REQUIRED)

add_executable(replay_vio replay_vio.cpp ${HSV2_CPP}/recorder.cpp)
target_include_directories(replay_vio PRIVATE
        ${HSV2_CPP}
        ${OPENVINS_SRC}/ov_core/src
        ${OPENVINS_SRC}/ov_init/src
        ${OPENVINS_SRC}/ov_msckf/src
        ${EIGEN3_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS})
target_link_libraries(replay_vio PRIVATE
        ${OPENVINS_BUILD}/libov_msckf_lib.so
        ${OpenCV_LIBS} Boost::system Boost::filesystem Boost::thread Boost::date_time
        Ceres::ceres Threads::Threads)
set_target_properties(replay_vio PROPERTIES BUILD_RPATH "${OPENVINS_BUILD}")
