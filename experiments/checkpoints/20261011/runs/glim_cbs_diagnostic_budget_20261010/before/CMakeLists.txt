cmake_minimum_required(VERSION 3.16)
project(glim_ros VERSION 1.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

option(BUILD_WITH_CUDA "Build with GPU support" ON)
option(BUILD_WITH_VIEWER "Build with visualizer" ON)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  tf2_ros
  roslib
  rosbag
  sensor_msgs
  nav_msgs
  geometry_msgs
  rosgraph_msgs
  liorf
)

find_package(glim REQUIRED)
find_package(gtsam_points REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED core)
find_package(GTSAM REQUIRED)
find_package(spdlog REQUIRED)
find_package(cbs REQUIRED)
find_package(aria_viz REQUIRED)

if(TARGET gtsam_eigen3)
  set(GLIM_ROS_EIGEN_TARGET gtsam_eigen3)
else()
  set(GLIM_ROS_EIGEN_TARGET Eigen3::Eigen)
endif()

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES glim_ros
  CATKIN_DEPENDS
    cv_bridge
    image_transport
    tf2_ros
    roslib
    rosbag
    sensor_msgs
    nav_msgs
    geometry_msgs
    rosgraph_msgs
    liorf
#  DEPENDS glim
)

###########
## Build ##
###########

if(BUILD_WITH_VIEWER)
  add_definitions(-DBUILD_WITH_VIEWER)
endif()

if(BUILD_WITH_CUDA)
  add_definitions(-DBUILD_GTSAM_POINTS_GPU)
endif()

# if glim::glim does not exist
if (NOT TARGET glim::glim)
  add_library(glim::glim INTERFACE IMPORTED GLOBAL)
  set_target_properties(glim::glim PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${glim_INCLUDE_DIRS}"
    INTERFACE_LINK_LIBRARIES "${glim_LIBRARIES}"
  )

  add_library(glim::interactive_viewer INTERFACE IMPORTED GLOBAL)
  set_target_properties(glim::interactive_viewer PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${glim_INCLUDE_DIRS}"
    INTERFACE_LINK_LIBRARIES interactive_viewer
  )
endif()


add_library(glim_ros
  src/glim_ros/glim_ros.cpp
  src/glim_ros/cbs_bridge.cpp
  src/glim_ros/dcreg_edge_metadata_core.cpp
  src/glim_ros/dcreg_edge_metadata_publisher.cpp
  src/glim_ros/sha256.cpp
  src/glim_ros/glim_factor_graph_inspector.cpp
)
add_dependencies(glim_ros ${catkin_EXPORTED_TARGETS})
target_include_directories(glim_ros PUBLIC
  include
  ${catkin_INCLUDE_DIRS}
)
set(GTSAM_BUNDLED_EIGEN_INCLUDE_DIR
  ${GTSAM_INCLUDE_DIR}/gtsam/3rdparty/Eigen)
if(EXISTS ${GTSAM_BUNDLED_EIGEN_INCLUDE_DIR})
  target_include_directories(glim_ros SYSTEM BEFORE PUBLIC
    ${GTSAM_BUNDLED_EIGEN_INCLUDE_DIR}
  )
endif()
target_link_libraries(glim_ros
  glim::glim
  gtsam_points::gtsam_points
  ${GLIM_ROS_EIGEN_TARGET}
  ${catkin_LIBRARIES}
  cbs
  aria_viz
)
target_compile_definitions(glim_ros PUBLIC GLIM_USE_OPENCV)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(cbs_native_imu_mean_bridge_test
    test/cbs_native_imu_mean_bridge_test.cpp
  )
  if(TARGET cbs_native_imu_mean_bridge_test)
    target_link_libraries(cbs_native_imu_mean_bridge_test glim_ros gtest_main)
  endif()
  catkin_add_gtest(dcreg_edge_metadata_test
    test/dcreg_edge_metadata_test.cpp
  )
  if(TARGET dcreg_edge_metadata_test)
    add_dependencies(dcreg_edge_metadata_test ${catkin_EXPORTED_TARGETS})
    target_link_libraries(dcreg_edge_metadata_test glim_ros gtest_main)
  endif()
endif()

add_library(rviz_viewer SHARED
  src/glim_ros/rviz_viewer.cpp
)
target_link_libraries(rviz_viewer
  glim_ros
)

add_executable(glim_rosnode
  src/glim_rosnode.cpp
)
target_link_libraries(glim_rosnode
  glim_ros
)

add_executable(glim_rosbag
  src/glim_rosbag.cpp
)
target_link_libraries(glim_rosbag
  glim_ros
)


if(BUILD_WITH_VIEWER)
  add_executable(offline_viewer
    src/offline_viewer.cpp
  )
  target_link_libraries(offline_viewer
    glim::interactive_viewer
    glim_ros
  )
endif()

#############
## Install ##
#############
install(DIRECTORY include/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

set(GLIM_ROS_INSTALL_TARGETS
  glim_ros
  rviz_viewer
  glim_rosnode
  glim_rosbag
)

if(BUILD_WITH_VIEWER)
  list(APPEND GLIM_ROS_INSTALL_TARGETS offline_viewer)
endif()

install(TARGETS ${GLIM_ROS_INSTALL_TARGETS}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
