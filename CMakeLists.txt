cmake_minimum_required(VERSION 3.0.2)
project(turtle_lawnmower)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  turtlesim
)

catkin_package()
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(turtle_lawnmower_node src/turtle_lawnmower_node.cpp)

target_link_libraries(turtle_lawnmower_node
	${catkin_LIBRARIES}
)

