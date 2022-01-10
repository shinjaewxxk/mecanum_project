cmake_minimum_required(VERSION 2.8.3)
project(mecanum1)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

## Declare ROS messages and services
#add_message_files(test.msg)
#add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

#add_executable(rc_to_pwm src/rc_to_pwm.cpp)
#target_link_libraries(rc_to_pwm ${catkin_LIBRARIES})
#add_dependencies(rc_to_pwm mecanum1_generate_messages_cpp)

#add_executable(rc_to_pwm_sj src/rc_to_pwm_sj.cpp)
#target_link_libraries(rc_to_pwm_sj ${catkin_LIBRARIES})
#add_dependencies(rc_to_pwm_sj mecanum1_generate_messages_cpp)

#add_executable(rc_to_pwm_clean src/rc_to_pwm_clean.cpp)
#target_link_libraries(rc_to_pwm_clean ${catkin_LIBRARIES})
#add_dependencies(rc_to_pwm_clean mecanum1_generate_messages_cpp)

#add_executable(rc_to_pwm_ex src/rc_to_pwm_ex.cpp)
#target_link_libraries(rc_to_pwm_ex ${catkin_LIBRARIES})
#add_dependencies(rc_to_pwm_ex mecanum1_generate_messages_cpp)

#add_executable(rc_to_pwm_ex_1 src/rc_to_pwm_ex_1.cpp)
#target_link_libraries(rc_to_pwm_ex_1 ${catkin_LIBRARIES})
#add_dependencies(rc_to_pwm_ex_1 mecanum1_generate_messages_cpp)

add_executable(rc_to_pwm_sbus src/rc_to_pwm_sbus.cpp)
target_link_libraries(rc_to_pwm_sbus ${catkin_LIBRARIES})
add_dependencies(rc_to_pwm_sbus mecanum1_generate_messages_cpp)

#add_executable(rc_to_pwm_v_m src/rc_to_pwm_v_m.cpp)
#target_link_libraries(rc_to_pwm_v_m ${catkin_LIBRARIES})
#add_dependencies(rc_to_pwm_v_m mecanum1_generate_messages_cpp)

