# Our own programs on top of a built ORB-SLAM3 (mono_bench, later mono_live).
#   cmake -S apps -B apps/build -DORB_SLAM3_ROOT=$HOME/orbslam/orbslam3 -DOpenCV_DIR=... -DCMAKE_PREFIX_PATH=$HOME/local && cmake --build apps/build -j4
cmake_minimum_required(VERSION 3.10)
project(orbslam3_apps)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
set(ORB_SLAM3_ROOT "$ENV{HOME}/orbslam/orbslam3" CACHE PATH "built ORB-SLAM3 tree")
find_package(OpenCV 4.4 REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization)
include_directories(${ORB_SLAM3_ROOT} ${ORB_SLAM3_ROOT}/include ${ORB_SLAM3_ROOT}/include/CameraModels
                    ${ORB_SLAM3_ROOT}/Thirdparty/Sophus ${EIGEN3_INCLUDE_DIR} ${Pangolin_INCLUDE_DIRS})
set(ORB_LIBS ${ORB_SLAM3_ROOT}/lib/libORB_SLAM3.so ${ORB_SLAM3_ROOT}/Thirdparty/DBoW2/lib/libDBoW2.so
             ${ORB_SLAM3_ROOT}/Thirdparty/g2o/lib/libg2o.so ${OpenCV_LIBS} ${Pangolin_LIBRARIES} ${Boost_LIBRARIES} OpenSSL::Crypto)
# Find Pangolin's libraries at run time without LD_LIBRARY_PATH. Plain RUNPATH is not used for the dependencies of
# dependencies (libpango_display needs libpango_windowing), so use the old-style RPATH, which is.
get_filename_component(PANGOLIN_LIBDIR "${Pangolin_DIR}/../.." ABSOLUTE)
set(CMAKE_BUILD_RPATH "${PANGOLIN_LIBDIR};${ORB_SLAM3_ROOT}/lib;${ORB_SLAM3_ROOT}/Thirdparty/DBoW2/lib;${ORB_SLAM3_ROOT}/Thirdparty/g2o/lib")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--disable-new-dtags")
add_executable(mono_bench mono_bench.cc)
target_link_libraries(mono_bench ${ORB_LIBS})
add_executable(mono_live mono_live.cc)
target_link_libraries(mono_live ${ORB_LIBS} pthread)
