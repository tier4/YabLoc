cmake_minimum_required(VERSION 3.5)
project(undistort)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # add_compile_options(-Wall -Wextra -Wpedantic -Werror)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ===================================================
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# ===================================================
# OpenCV
find_package(OpenCV REQUIRED)

# ===================================================
# Executable
set(TARGET undistort_node)
ament_auto_add_executable(${TARGET} src/undistort_node.cpp)
target_link_libraries(${TARGET} ${OpenCV_LIBS})

# ===================================================
ament_auto_package(INSTALL_TO_SHARE launch)