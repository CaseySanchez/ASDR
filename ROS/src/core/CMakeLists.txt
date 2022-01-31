# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 2.8.3)
project(core)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

set(cpprestsdk_DIR /usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}/cmake/)

## Find CPPRESTSDK
find_package(cpprestsdk REQUIRED)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include HFSM2/include ${catkin_INCLUDE_DIRS})

# %Tag(SRVCLIENT)%

add_executable(core_node src/core_node.cpp src/state_machine.cpp)
target_link_libraries(core_node ${catkin_LIBRARIES} cpprestsdk::cpprest)
add_dependencies(core_node core_gencpp ${catkin_EXPORTED_TARGETS})

# %EndTag(SRVCLIENT)%

# %EndTag(FULLTEXT)%

