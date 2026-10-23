# OpenVINS (GPL-3.0) for Android arm64, as a static library target "openvins".
# Core + static initialiser + MSCKF only: no ROS, simulator, tests, or dynamic
# initialiser (upstream disables it on __ANDROID__, so Ceres is not needed).
# Include from a CMakeLists after setting OPENVINS_SRC, EIGEN3_DIR, BOOST_HEADERS, OpenCV_DIR.
set(OV_COMPAT "${CMAKE_CURRENT_LIST_DIR}/compat")

file(GLOB_RECURSE OV_CORE_SRC "${OPENVINS_SRC}/ov_core/src/*.cpp")
file(GLOB_RECURSE OV_INIT_SRC "${OPENVINS_SRC}/ov_init/src/*.cpp")
file(GLOB_RECURSE OV_MSCKF_SRC "${OPENVINS_SRC}/ov_msckf/src/*.cpp")
set(OV_SRC ${OV_CORE_SRC} ${OV_INIT_SRC} ${OV_MSCKF_SRC})
# Drop ROS nodes, simulator, tests, executables, dynamic init (Ceres).
list(FILTER OV_SRC EXCLUDE REGEX "/ros/|/sim/|/test_[^/]*\\.cpp$|/run_[^/]*\\.cpp$|ros1_serial|dummy\\.cpp$|/dynamic/|/ceres/|/plot/")

find_package(OpenCV 4 REQUIRED CONFIG)

add_library(openvins STATIC ${OV_SRC})
target_include_directories(openvins PUBLIC
        "${OV_COMPAT}"
        "${OPENVINS_SRC}/ov_core/src"
        "${OPENVINS_SRC}/ov_init/src"
        "${OPENVINS_SRC}/ov_msckf/src"
        "${EIGEN3_DIR}"
        "${BOOST_HEADERS}"
        ${OpenCV_INCLUDE_DIRS})
target_compile_definitions(openvins PUBLIC BOOST_ALL_NO_LIB BOOST_DATE_TIME_NO_LIB)
# The Boost 1.7x MPL headers trip clang >= 16's enum range check (an error by default);
# needed by every target that includes OpenVINS headers.
target_compile_options(openvins PUBLIC -Wno-enum-constexpr-conversion)
target_compile_options(openvins PRIVATE -O3 -fexceptions -frtti -Wno-deprecated-declarations)
target_link_libraries(openvins PUBLIC ${OpenCV_LIBS} log)
