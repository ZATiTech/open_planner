cmake_minimum_required(VERSION 3.5)
project(op_fix2pose)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()


if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-gnu-anonymous-struct -Wno-nested-anon-types )
endif()

# set(CMAKE_BUILD_TYPE "Release")

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()


find_package(Boost COMPONENTS signals)

# Resolve system dependency on yaml-cpp, which apparently does not
# provide a CMake find_package() module.
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIR
          NAMES yaml_cpp.h
          PATHS ${YAML_CPP_INCLUDE_DIRS})
find_library(YAML_CPP_LIBRARY
             NAMES YAML_CPP
             PATHS ${YAML_CPP_LIBRARY_DIRS})

link_directories(${YAML_CPP_LIBRARY_DIRS})

if(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")
  add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")

include_directories(    
)

ament_auto_add_library(op_fix2pose SHARED
  src/op_fix2pose/op_fix2pose_node.cpp
)


rclcpp_components_register_node(op_fix2pose 
  PLUGIN "Fix2PoseInterface"
  EXECUTABLE op_fix2pose_node
)


ament_auto_package(
  INSTALL_TO_SHARE
    launch
)

