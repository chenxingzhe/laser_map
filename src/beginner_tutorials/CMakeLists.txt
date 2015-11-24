# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 2.8.3)
project(beginner_tutorials)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg sensor_msgs message_filters tf
  tf2_sensor_msgs)
  
find_package(Boost REQUIRED)

## Declare ROS messages and services
##add_message_files(FILES Num.msg)
##add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs sensor_msgs)

## Declare a catkin package
catkin_package( CATKIN_DEPENDS message_runtime
tf message_filters)
## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})


add_executable(talker src/talker.cpp
src/lsd.cpp
src/lsd_lines.cpp
src/lsd_opencv.cpp
src/lsd_wrap.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})

 find_package(OpenCV REQUIRED)  
 target_link_libraries(talker  ${OpenCV_LIBS} ${Boost_LIBRARY_DIRS})
add_dependencies(talker beginner_tutorials_generate_messages_cpp)

add_executable(calib src/calib.cpp)
target_link_libraries(calib ${catkin_LIBRARIES})
 target_link_libraries(calib  ${OpenCV_LIBS} ${Boost_LIBRARY_DIRS})
 add_dependencies(calib beginner_tutorials_generate_messages_cpp)
 
 
add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
add_dependencies(listener beginner_tutorials_generate_messages_cpp)

# %EndTag(FULLTEXT)%

