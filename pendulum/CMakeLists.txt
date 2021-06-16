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

include_directories(${GAZEBO_INCLUDE_DIRS} include)
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(util src/util.cpp)
target_link_libraries(util ${catkin_LIBRARIES})

add_executable(my_controller src/main.cpp)
target_link_libraries(my_controller ${catkin_LIBRARIES} util)

add_executable(prism_ctrl src/prism_ctrl.cpp)
target_link_libraries(prism_ctrl ${catkin_LIBRARIES} util)

add_executable(base_contact src/base_contact.cpp)
target_link_libraries(base_contact ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} pthread util)

add_executable(spring_emulator src/spring_emulator.cpp)
target_link_libraries(spring_emulator ${catkin_LIBRARIES} util)