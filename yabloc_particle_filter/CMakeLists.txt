cmake_minimum_required(VERSION 3.5)
project(yabloc_particle_filter)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# ===================================================
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# ===================================================
# PCL
find_package(PCL REQUIRED)

# Sophus
find_package(Sophus REQUIRED)

# ===================================================
# Message & Service
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Particle.msg"
  "msg/ParticleArray.msg"
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

# ===================================================
# Libraries
ament_auto_add_library(
  abst_corrector
  SHARED
  src/correction/abst_corrector.cpp
  src/common/visualize.cpp
  src/common/mean.cpp
)
target_include_directories(abst_corrector SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(abst_corrector Sophus::Sophus ${PCL_LIBRARIES})
rosidl_target_interfaces(abst_corrector ${PROJECT_NAME} "rosidl_typesupport_cpp")

ament_auto_add_library(predictor
  src/prediction/predictor.cpp
  src/prediction/resampler.cpp
  src/common/visualize.cpp
  src/common/mean.cpp
)
target_include_directories(predictor SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(predictor Sophus::Sophus ${PCL_LIBRARIES})
rosidl_target_interfaces(predictor ${PROJECT_NAME} "rosidl_typesupport_cpp")

# ===================================================
# Executables
# Predictor
ament_auto_add_executable(predictor_node
  src/prediction/predictor_node.cpp
)
target_link_libraries(predictor_node predictor)
rosidl_target_interfaces(predictor_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Visualizer
ament_auto_add_executable(particle_visualize
  src/common/particle_visualize_node.cpp
)
rosidl_target_interfaces(particle_visualize ${PROJECT_NAME} "rosidl_typesupport_cpp")

# ===================================================
# TEST
if(BUILD_TESTING)
  add_subdirectory(test)
endif()

# ===================================================
install(PROGRAMS
  script/particle_array_to_marker_array.py
  script/particle_array_to_pose_array.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(INSTALL_TO_SHARE config)