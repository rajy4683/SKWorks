
set( CMAKE_DEEP_FACTORS_SRC_DIR "../sources")
#set( CMAKE_DEEP_FACTORS_TP_DIR "../thirdparty")
#add_subdirectory(${CMAKE_DEEP_FACTORS_TP_DIR} "thirdparty")
#add_subdirectory(${CMAKE_DEEP_FACTORS_TP_DIR}/Sophus "thirdparty")
#add_subdirectory(${CMAKE_DEEP_FACTORS_TP_DIR}/vision_core "thirdparty")

find_package(OpenCV REQUIRED)
find_package(Sophus REQUIRED)
find_package(VisionCore REQUIRED)
find_package(DBoW2 REQUIRED)
find_package(brisk        QUIET REQUIRED)
#find_package(Glog         QUIET REQUIRED)
#find_package(OpenCV       QUIET REQUIRED)
#find_package(opengv       QUIET REQUIRED)
#find_package(GTSAM        QUIET REQUIRED)
#find_package(Eigen3       QUIET REQUIRED)
#find_package(Pangolin     QUIET REQUIRED)
#find_package(Boost        QUIET REQUIRED COMPONENTS filesystem)
#find_package(TensorFlow   QUIET REQUIRED)

set(simple_test_sources
  simple_slam_test.cpp
)

set(common_sources
  test_pinhole_camera.cpp
)
message("${CMAKE_CURRENT_SOURCE_DIR}")

set(common_headers
  # algorithm
  algorithm/pinhole_camera_impl.h
  #algorithm/m_estimators.h
  #algorithm/dense_sfm.h
  algorithm/camera_pyramid.h
  #algorithm/warping.h
  #algorithm/pinhole_camera.h
  #algorithm/nearest_psd.h
  #algorithm/lucas_kanade_se3.h
  # top-level
  #image_sequences.h
  #indexed_map.h
  #tum_io.h
  #interp.h
  #timing.h
  #display_utils.h
  #logutils.h
  #intrinsics.h
  drivers/dataset/dataset_interface.h
  drivers/dataset/tum_interface.h
  drivers/dataset/camera_interface_factory.h
  drivers/camera_interface_factory.h
  common/tum_io.h
)
#include_directories("/usr/local/include/eigen3")

# ------------------------------------------------------------------------------
# Build target
# ------------------------------------------------------------------------------
#add_library(df_common SHARED ${common_sources} ${common_headers})
#add_executable(rekog_pinhole_cam ${common_sources} ${common_headers})
add_executable(rekog_pinhole_cam  ${common_sources}) #${common_headers})
target_include_directories(rekog_pinhole_cam PUBLIC ${CMAKE_DEEP_FACTORS_SRC_DIR}
#  ${CMAKE_CURRENT_SOURCE_DIR}
#  ${CMAKE_CURRENT_SOURCE_DIR}/algorithm
  ${CMAKE_DEEP_FACTORS_SRC_DIR}/drivers/dataset
  ${CMAKE_DEEP_FACTORS_SRC_DIR}/drivers
  ${CMAKE_DEEP_FACTORS_SRC_DIR}/common
  ${CMAKE_DEEP_FACTORS_SRC_DIR}/core/features
  ${CMAKE_DEEP_FACTORS_SRC_DIR}/core/network
  ${CMAKE_DEEP_FACTORS_SRC_DIR}/common/algorithm
  ${CMAKE_DEEP_FACTORS_SRC_DIR}/cuda
  ${CMAKE_DEEP_FACTORS_SRC_DIR}/core/mapping
  ${CMAKE_DEEP_FACTORS_SRC_DIR}/core
)

add_executable(simple_slam_test  ${simple_test_sources}) #${common_headers})
target_include_directories(simple_slam_test PUBLIC ${CMAKE_DEEP_FACTORS_SRC_DIR}
#  ${CMAKE_CURRENT_SOURCE_DIR}
#  ${CMAKE_CURRENT_SOURCE_DIR}/algorithm
  ${CMAKE_DEEP_FACTORS_SRC_DIR}/drivers/dataset
  ${CMAKE_DEEP_FACTORS_SRC_DIR}/drivers
  ${CMAKE_DEEP_FACTORS_SRC_DIR}/common
  ${CMAKE_DEEP_FACTORS_SRC_DIR}/core/features
  ${CMAKE_DEEP_FACTORS_SRC_DIR}/core/network
  ${CMAKE_DEEP_FACTORS_SRC_DIR}/common/algorithm
  ${CMAKE_DEEP_FACTORS_SRC_DIR}/cuda
  ${CMAKE_DEEP_FACTORS_SRC_DIR}/core/mapping
  ${CMAKE_DEEP_FACTORS_SRC_DIR}/core
)

target_link_libraries(rekog_pinhole_cam PUBLIC
   Eigen3::Eigen
   Sophus::Sophus
   VisionCore
   opencv_core
   opencv_highgui
   jsoncpp
   DBoW2
   boost_filesystem
   boost_system
   glog
   gflags
   brisk
   ${Pangolin_LIBRARY}
   df_cuda
   df_core
   df_common
   df_drivers
)

target_link_libraries(simple_slam_test PUBLIC
   Eigen3::Eigen
   Sophus::Sophus
   VisionCore
   opencv_core
   opencv_highgui
   jsoncpp
   DBoW2
   boost_filesystem
   boost_system
   glog
   gflags
   brisk
   ${Pangolin_LIBRARY}
   df_cuda
   df_core
   df_common
   df_drivers
)

#target_include_directories(df_demo PUBLIC
#  ${CMAKE_CURRENT_SOURCE_DIR}
#)
#target_link_libraries(df_demo PUBLIC
#  df_core
#  df_gui
#  df_drivers
#  df_common
#  opencv_core
#  opencv_highgui
#  boost_filesystem
#  gflags
#  glog
#)
#find_package(brisk        QUIET REQUIRED)
#find_package(Glog         QUIET REQUIRED)
#find_package(OpenCV       QUIET REQUIRED)
#find_package(opengv       QUIET REQUIRED)
#find_package(GTSAM        QUIET REQUIRED)
#find_package(Eigen3       QUIET REQUIRED)
#find_package(Pangolin     QUIET REQUIRED)
#find_package(Boost        QUIET REQUIRED COMPONENTS filesystem)
#find_package(TensorFlow   QUIET REQUIRED)

