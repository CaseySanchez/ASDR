# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 2.8.3)
project(berry_imu)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

# %Tag(SRVCLIENT)%

add_executable(berry_imu_node src/berry_imu_node.cpp src/berry_imu.cpp)
target_link_libraries(berry_imu_node ${catkin_LIBRARIES})
add_dependencies(berry_imu_node berry_imu_gencpp)

# %EndTag(SRVCLIENT)%

# %EndTag(FULLTEXT)%

