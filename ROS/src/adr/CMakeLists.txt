# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 3.8)
project(adr)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

add_service_files(FILES get_state.srv set_state.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include HFSM2/include ${catkin_INCLUDE_DIRS})

# %Tag(SRVCLIENT)%

add_executable(adr_node src/adr_node.cpp src/state_machine.cpp)
target_link_libraries(adr_node ${catkin_LIBRARIES} Threads::Threads)
add_dependencies(adr_node adr_gencpp ${catkin_EXPORTED_TARGETS})
set_property(TARGET adr_node PROPERTY CXX_STANDARD 17)

# %EndTag(SRVCLIENT)%

# %EndTag(FULLTEXT)%

