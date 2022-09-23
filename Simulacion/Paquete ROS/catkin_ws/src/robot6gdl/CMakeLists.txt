cmake_minimum_required(VERSION 3.0.2)
project(robot6gdl)

find_package(catkin REQUIRED COMPONENTS
  robot_state_publisher
  roscpp
  rospy
  tf
  std_msgs
  sensor_msgs
)
include_directories(include ${catkin_INCLUDE_DIRS})


add_executable(movLineal src/movLineal.cpp src/gTrayArt.cpp src/claseRobots.cpp)
target_link_libraries(movLineal ${catkin_LIBRARIES})

add_executable(movCircular src/movCircular.cpp src/gTrayArt.cpp src/claseRobots.cpp)
target_link_libraries(movCircular ${catkin_LIBRARIES})

add_executable(movCicloidal src/movCicloidal.cpp src/gTrayArt.cpp src/claseRobots.cpp)
target_link_libraries(movCicloidal ${catkin_LIBRARIES})
