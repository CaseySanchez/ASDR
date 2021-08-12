# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 2.8.3)
project(adr)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

## Find wiringPi
find_library(wiringPi_LIB wiringPi)

set(cpprestsdk_DIR /usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}/cmake/)

## Find CPPRESTSDK
find_package(cpprestsdk REQUIRED)

## Find Threads
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

## Declare ROS messages and services
## add_message_files(FILES gyr.msg acc.msg mag.msg)
## add_service_files(FILES get_mode.srv set_mode.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS} ${WIRINGPI_INCLUDE_DIRS})

# %Tag(SRVCLIENT)%

add_executable(berry_imu_node src/berry_imu_node.cpp src/berry_imu.cpp)
target_link_libraries(berry_imu_node ${catkin_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} ${wiringPi_LIB} Threads::Threads cpprestsdk::cpprest)
add_dependencies(berry_imu_node adr_gencpp)

# %EndTag(SRVCLIENT)%

# %EndTag(FULLTEXT)%

