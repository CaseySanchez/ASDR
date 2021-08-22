# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 2.8.3)
project(path_planner)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

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

add_executable(path_planner_node src/path_planner_node.cpp src/mandoline.cpp)
target_link_libraries(path_planner_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} Eigen3::Eigen)
add_dependencies(path_planner_node path_planner_gencpp)

# %EndTag(SRVCLIENT)%

# %EndTag(FULLTEXT)%

