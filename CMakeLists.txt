cmake_minimum_required(VERSION 3.0.2)
project(pendulum)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)

catkin_package()

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})

## gazebo

find_package(gazebo REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_executable(my_controller src/main.cpp)
target_link_libraries(my_controller ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} pthread)

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} pthread)
