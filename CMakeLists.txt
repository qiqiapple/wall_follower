cmake_minimum_required(VERSION 2.8.3)
project(wall_follower)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  ras_arduino_msgs
  geometry_msgs
  message_generation
)

add_service_files(
	FILES
	MakeTurn.srv
        FollowWall.srv
)

generate_messages(
	DEPENDENCIES
	std_msgs
)

catkin_package(
	CATKIN_DEPENDS message_runtime roscpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(maze_follower src/maze_follower.cpp)
add_dependencies(maze_follower ${catkin_EXPORTED_TARGETS})
target_link_libraries(maze_follower ${catkin_LIBRARIES})

add_executable(turn_controller src/turn_controller.cpp)
add_dependencies(turn_controller ${catkin_EXPORTED_TARGETS})
target_link_libraries(turn_controller ${catkin_LIBRARIES})

add_executable(wall_follower src/wall_following_controller.cpp)
add_dependencies(wall_follower ${catkin_EXPORTED_TARGETS})
target_link_libraries(wall_follower ${catkin_LIBRARIES})
