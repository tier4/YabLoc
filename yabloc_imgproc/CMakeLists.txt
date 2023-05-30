cmake_minimum_required(VERSION 3.5)
project(yabloc_imgproc)

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
# Eigen3
find_package(Eigen3 REQUIRED)

# OpenCV
find_package(OpenCV REQUIRED)

# PCL
find_package(PCL REQUIRED COMPONENTS common)

# Sophus
find_package(Sophus REQUIRED)

# https://github.com/strasdat/Sophus/issues/388
# Sophus's macro cause compile error for the "-Wpedantic" flag, so make sure to import as a system.
# We must always call the following command after "ament_auto_find_build_dependencies()" because it rewrite CMAKE_NO_SYSTEM_FROM_IMPORTED to TRUE.
set(CMAKE_NO_SYSTEM_FROM_IMPORTED FALSE)

# message(STATUS "debug ${_ARG_REQUIRED}")
# message(STATUS "debug ${${PROJECT_NAME}_FOUND_INCLUDE_DIRS}")
# message(STATUS "debug ${${PROJECT_NAME}_FOUND_LIBRARIES}")
# message(STATUS "debug ${${PROJECT_NAME}_LIBRARIES}")
message(STATUS "debug ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS}")
message(STATUS "debug ${yabloc_common_LIBRARIES}")
message(STATUS "debug2 ${yabloc_common_TARGETS}")
message(STATUS "debug2 ${yabloc_common_INTERFACES}")
message(STATUS "debug2 ${ublox_msgs_TARGETS}")
message(STATUS "debug2 ${ublox_msgs_INTERFACES}")

# ===================================================
# Executable
# line segment detection
set(TARGET lsd_node)
ament_auto_add_executable(${TARGET}
  src/lsd/lsd_node.cpp
  src/lsd/lsd_core.cpp)
target_include_directories(${TARGET} PUBLIC include)
target_include_directories(${TARGET} PRIVATE SYSTEM ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
target_link_libraries(${TARGET} ${OpenCV_LIBS})

# target_link_libraries(${TARGET} /home/kentoyabuuchi2/Projects/vmvl_ws/install/yabloc_common/lib/libyabloc_common_lib.so)
function(original_ament_target_dependencies target)
  if(NOT TARGET ${target})
    message(FATAL_ERROR "ament_target_dependencies() the first argument must be a valid target name")
  endif()

  if(${ARGC} GREATER 0)
    cmake_parse_arguments(ARG "INTERFACE;PUBLIC;SYSTEM" "" "" ${ARGN})
    set(ARGVIND 1)
    set(system_keyword "")
    set(optional_keyword "")
    set(required_keyword "PUBLIC")

    if(ARG_SYSTEM)
      if(NOT "${ARGV${ARGVIND}}" STREQUAL "SYSTEM")
        message(FATAL_ERROR "ament_target_dependencies() SYSTEM keyword is only allowed before the package names and other keywords")
      endif()

      set(system_keyword SYSTEM)
      math(EXPR ARGVIND "${ARGVIND} + 1")
    endif()

    if(ARG_INTERFACE)
      if(NOT "${ARGV${ARGVIND}}" STREQUAL "INTERFACE")
        message(FATAL_ERROR "ament_target_dependencies() INTERFACE keyword is only allowed before the package names")
      endif()

      set(optional_keyword INTERFACE)
      set(required_keyword INTERFACE)
    endif()

    if(ARG_PUBLIC)
      if(NOT "${ARGV${ARGVIND}}" STREQUAL "PUBLIC")
        message(FATAL_ERROR "ament_target_dependencies() PUBLIC keyword is only allowed before the package names")
      endif()

      set(optional_keyword PUBLIC)
    endif()

    set(definitions "")
    set(include_dirs "")
    set(interfaces "")
    set(libraries "")
    set(link_flags "")

    foreach(package_name ${ARG_UNPARSED_ARGUMENTS})
      if(NOT "${${package_name}_FOUND}")
        message(FATAL_ERROR "ament_target_dependencies() the passed package name '${package_name}' was not found before")
      endif()

      # if a package provides modern CMake interface targets use them
      # exclusively assuming the classic CMake variables only exist for
      # backward compatibility
      set(use_modern_cmake FALSE)

      if(NOT "${${package_name}_TARGETS}" STREQUAL "")
        foreach(_target ${${package_name}_TARGETS})
          # only use actual targets
          # in case a package uses this variable for other content
          if(TARGET "${_target}")
            list_append_unique(interfaces ${_target})
            set(use_modern_cmake TRUE)
          endif()
        endforeach()
      endif()

      if(NOT use_modern_cmake AND NOT "${${package_name}_INTERFACES}" STREQUAL "")
        foreach(_interface ${${package_name}_INTERFACES})
          # only use actual targets
          # in case a package uses this variable for other content
          if(TARGET "${_interface}")
            list_append_unique(interfaces ${_interface})
            set(use_modern_cmake TRUE)
          endif()
        endforeach()

        if(use_modern_cmake)
          message(DEPRECATION
            "Package ${package_name} is exporting the variable "
            "${package_name}_INTERFACES which is deprecated, it should export
            ${package_name}_TARGETS instead")
        endif()
      endif()

      if(NOT use_modern_cmake)
        # otherwise use the classic CMake variables
        list_append_unique(definitions ${${package_name}_DEFINITIONS})
        list_append_unique(include_dirs ${${package_name}_INCLUDE_DIRS})

        foreach(library ${${package_name}_LIBRARIES})
          message(STATUS "test ${library}")

          if(NOT "${${package_name}_LIBRARY_DIRS}" STREQUAL "")
            if(NOT IS_ABSOLUTE ${library} OR NOT EXISTS ${library})
              find_library(lib NAMES ${library} PATHS ${${package_name}_LIBRARY_DIRS} NO_DEFAULT_PATH)

              if(lib)
                set(library ${lib})
              endif()
            endif()
          endif()

          list(APPEND libraries ${library})
        endforeach()

        list_append_unique(link_flags ${${package_name}_LINK_FLAGS})
      endif()
    endforeach()

    if(NOT ARG_INTERFACE)
      target_compile_definitions(${target}
        ${required_keyword} ${definitions})

      # the interface include dirs must be ordered
      set(interface_include_dirs)

      foreach(interface ${interfaces})
        get_target_property(_include_dirs ${interface} INTERFACE_INCLUDE_DIRECTORIES)

        if(_include_dirs)
          list_append_unique(interface_include_dirs ${_include_dirs})
        endif()
      endforeach()

      ament_include_directories_order(ordered_interface_include_dirs ${interface_include_dirs})

      # the interface include dirs are used privately to ensure proper order
      # and the interfaces cover the public case
      target_include_directories(${target} ${system_keyword}
        PRIVATE ${ordered_interface_include_dirs})
    endif()

    ament_include_directories_order(ordered_include_dirs ${include_dirs})
    target_link_libraries(${target}
      ${optional_keyword} ${interfaces})
    target_include_directories(${target} ${system_keyword}
      ${required_keyword} ${ordered_include_dirs})

    if(NOT ARG_INTERFACE)
      message(STATUS "debug4 ${libraries}")
      ament_libraries_deduplicate(unique_libraries ${libraries})
      target_link_libraries(${target}
        ${optional_keyword} ${unique_libraries})
      message(STATUS "debug5: ${unique_libraries}")

      foreach(link_flag IN LISTS link_flags)
        set_property(TARGET ${target} APPEND_STRING PROPERTY LINK_FLAGS " ${link_flag} ")
      endforeach()
    endif()
  endif()
endfunction()

original_ament_target_dependencies(${TARGET} SYSTEM ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS})

# # graph based segmentation
# set(TARGET graph_segment_node)
# ament_auto_add_executable(${TARGET}
# src/graph_segment/graph_segment_node.cpp
# src/graph_segment/graph_segment_core
# src/graph_segment/similar_area_searcher.cpp)
# target_include_directories(${TARGET} PUBLIC include)
# target_include_directories(${TARGET} SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIRS})
# target_link_libraries(${TARGET} ${OpenCV_LIBS})

# # segment filter
# set(TARGET segment_filter_node)
# ament_auto_add_executable(${TARGET}
# src/segment_filter/segment_filter_node.cpp
# src/segment_filter/segment_filter_core.cpp)
# target_include_directories(${TARGET} PUBLIC include)
# target_include_directories(${TARGET} PRIVATE SYSTEM ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
# target_link_libraries(${TARGET} ${PCL_LIBRARIES} ${OpenCV_LIBS})

# # undistort
# set(TARGET undistort_node)
# ament_auto_add_executable(${TARGET}
# src/undistort/undistort_node.cpp)
# target_link_libraries(${TARGET} ${OpenCV_LIBS})

# # line_segments_overlay
# set(TARGET line_segments_overlay_node)
# ament_auto_add_executable(${TARGET}
# src/line_segments_overlay/line_segments_overlay_core.cpp
# src/line_segments_overlay/line_segments_overlay_node.cpp)
# target_include_directories(${TARGET} PUBLIC include)
# target_include_directories(${TARGET} PRIVATE SYSTEM ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
# target_link_libraries(${TARGET} Sophus::Sophus ${PCL_LIBRARIES})

# # lanelet2_overlay
# set(TARGET lanelet2_overlay_node)
# ament_auto_add_executable(${TARGET}
# src/lanelet2_overlay/lanelet2_overlay_core.cpp
# src/lanelet2_overlay/lanelet2_overlay_node.cpp)
# target_include_directories(${TARGET} PUBLIC include)
# target_include_directories(${TARGET} PRIVATE SYSTEM ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
# target_link_libraries(${TARGET} Sophus::Sophus ${PCL_LIBRARIES})

# ===================================================
ament_auto_package(INSTALL_TO_SHARE config)