cmake_minimum_required(VERSION 2.8.3)
project(wall_follower)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  ras_arduino_msgs
  geometry_msgs
)



catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(maze_follower src/maze_follower.cpp)
add_dependencies(maze_follower ${catkin_EXPORTED_TARGETS})
target_link_libraries(maze_follower ${catkin_LIBRARIES})
