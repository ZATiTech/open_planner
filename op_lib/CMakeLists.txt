cmake_minimum_required(VERSION 3.14)
project(open_planner)

find_package(autoware_cmake REQUIRED)
autoware_package()

include_directories(
  include
)

ament_auto_add_library(openplanner SHARED
  src/utilities/utility.cpp
  src/utilities/data_rw.cpp
  src/utilities/matrix_operations.cpp
  src/planning/RoadNetwork.cpp
  src/planning/PlanningHelpers.cpp
  src/mapping/MappingHelpers.cpp
  src/hmi/HMIMSG.cpp
  
)

target_link_libraries(openplanner 
  tinyxml
  proj
)

target_include_directories(openplanner
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_auto_package(
  INSTALL_TO_SHARE
)
