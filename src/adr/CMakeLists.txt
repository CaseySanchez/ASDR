# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 2.8.3)
project(adr)

## Find catkin and any catkin packages
#find_package(catkin REQUIRED COMPONENTS tf roscpp rospy std_msgs genmsg)
find_package(catkin REQUIRED COMPONENTS smacc eg_conditional_generator tf roscpp rospy std_msgs genmsg)

find_package(PCL 1.10)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

add_definitions(${PCL_DEFINITIONS})
# %Tag(SRVCLIENT)%

add_executable(state_machine_node src/state_machine_node.cpp)
target_link_libraries(state_machine_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} Eigen3::Eigen)
add_dependencies(state_machine_node adr_gencpp)

add_executable(path_planner_node src/path_planner_node.cpp src/mandoline.cpp src/discovery.cpp)
target_link_libraries(path_planner_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} Eigen3::Eigen)
add_dependencies(path_planner_node adr_gencpp)
set_property(TARGET path_planner_node PROPERTY CXX_STANDARD 17)

# %EndTag(SRVCLIENT)%

# %EndTag(FULLTEXT)%

