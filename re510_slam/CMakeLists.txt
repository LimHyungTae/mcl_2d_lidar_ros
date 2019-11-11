cmake_minimum_required(VERSION 2.8.3)
project(re510_slam)
set (CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    geometry_msgs
    nav_msgs
    std_msgs
    sensor_msgs
    cv_bridge
)

catkin_package(
)


include_directories(
  tool
  ${catkin_INCLUDE_DIRS}
)

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

set(tool_source
    tool/tool.cpp
    tool/tool.h
)

# MAPGEN
set(rs_mapgen_source
    rs_mapgen/src/mapgen.cpp
    rs_mapgen/src/mapgen.h
)

add_executable(rs_mapgen rs_mapgen/rs_mapgen.cpp ${rs_mapgen_source} ${tool_source})
target_link_libraries(rs_mapgen ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# MCL
set(rs_mcl_source
    rs_mcl/src/mcl.cpp
    rs_mcl/src/mcl.h
)

add_executable(rs_mcl rs_mcl/rs_mcl.cpp ${rs_mcl_source} ${tool_source})
target_link_libraries(rs_mcl ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
