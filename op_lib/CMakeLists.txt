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
  src/mapping/opendrive/parser/GeometryParser.cpp
	src/mapping/opendrive/parser/pugixml.cpp
	src/mapping/opendrive/parser/GeoReferenceParser.cpp
	src/mapping/opendrive/parser/JunctionParser.cpp
	src/mapping/opendrive/parser/LaneParser.cpp
	src/mapping/opendrive/parser/OpenDriveParser.cpp
	src/mapping/opendrive/parser/ProfilesParser.cpp
	src/mapping/opendrive/parser/RoadLinkParser.cpp
	src/mapping/opendrive/parser/TrafficGroupParser.cpp
	src/mapping/opendrive/parser/TrafficSignalsParser.cpp
	src/mapping/opendrive/parser/TrafficSignParser.cpp
	src/mapping/opendrive/geometry/CenterLine.cpp
	src/mapping/opendrive/geometry/Geometry.cpp
	src/mapping/opendrive/geometry/GeometryGenerator.cpp
	src/mapping/opendrive/geometry/LaneUtils.cpp
  src/hmi/HMIMSG.cpp
  
)

target_link_libraries(openplanner
)

target_include_directories(openplanner
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_auto_package(
  INSTALL_TO_SHARE
)