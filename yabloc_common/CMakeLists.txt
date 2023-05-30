cmake_minimum_required(VERSION 3.5)
project(yabloc_common)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# ===================================================
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# ===================================================
# Eigen3
find_package(Eigen3 REQUIRED)

# OpenCV
find_package(OpenCV REQUIRED)

# PCL
find_package(PCL REQUIRED)

# Sophus
find_package(Sophus REQUIRED)

# https://github.com/strasdat/Sophus/issues/388
# Sophus's macro cause compile error for the "-Wpedantic" flag, so make sure to import as a system.
# We must always call the following command after "ament_auto_find_build_dependencies()" because it rewrite CMAKE_NO_SYSTEM_FROM_IMPORTED to TRUE.
set(CMAKE_NO_SYSTEM_FROM_IMPORTED FALSE)

# glog
find_package(glog REQUIRED)

# ===================================================
# GeographicLib
find_package(PkgConfig)
find_path(GeographicLib_INCLUDE_DIR GeographicLib/Config.h
  PATH_SUFFIXES GeographicLib
)
set(GeographicLib_INCLUDE_DIRS ${GeographicLib_INCLUDE_DIR})
find_library(GeographicLib_LIBRARIES NAMES Geographic)

# ===================================================
# Libraries
set(TARGET ${PROJECT_NAME}_lib)
file(GLOB REGULATORY_ELEMENT_SOURCE src/regulatory_elements/*cpp)
add_library(${TARGET}
  ${REGULATORY_ELEMENT_SOURCE}
  src/ublox_stamp.cpp
  src/pub_sub.cpp
  src/fix2mgrs.cpp
  src/camera_info_subscriber.cpp
  src/cv_decompress.cpp
  src/pose_conversions.cpp
  src/static_tf_subscriber.cpp
  src/extract_line_segments.cpp
  src/from_bin_msg.cpp
  src/transform_line_segments.cpp
  src/color.cpp)
target_include_directories(${TARGET} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_include_directories(
  ${TARGET} PRIVATE
  SYSTEM
  ${PCL_INCLUDE_DIRS}
  ${GeographicLib_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(${TARGET} Geographic ${PCL_LIBRARIES} Sophus::Sophus)
ament_target_dependencies(${TARGET} rclcpp std_msgs geometry_msgs tf2_ros sensor_msgs ublox_msgs cv_bridge pcl_conversions
  autoware_auto_mapping_msgs)

# ===================================================
# Clear ${PYTHON_EXECUTABLE} defined by mrt_cmake_module
unset(PYTHON_EXECUTABLE)
message(STATUS "PYTHON_EXECUTABLE: ${PYTHON_EXECUTABLE}")

# ===================================================
# Message & Service
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Ground.srv"
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

install(TARGETS ${PROJECT_NAME}_lib
  EXPORT ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION "include/${PROJECT_NAME}"
)

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(${PROJECT_NAME}_lib)
ament_export_targets(${PROJECT_NAME}_lib)
ament_export_dependencies(rosidl_default_runtime rclcpp std_msgs geometry_msgs tf2_ros sensor_msgs ublox_msgs cv_bridge pcl_conversions
  autoware_auto_mapping_msgs)
ament_export_dependencies(PCL Sophus)

ament_package()