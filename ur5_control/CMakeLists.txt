cmake_minimum_required(VERSION 3.0.2)
project(ur5_control)

find_package(catkin REQUIRED
    roscpp
    sensor_msgs
    geometry_msgs
    std_msgs
    trajectory_msgs
    message_generation
    control_msgs
    cartesian_state_msgs
    tf
    tf_conversions
    trajectory_msgs
    eigen_conversions
)

add_message_files(
  FILES
  ur5_pos.msg
  ur5_dke_pos.msg
)

generate_messages(
  DEPENDENCIES
  control_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ur5_control
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)


include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}/include

)

add_library(admittance_src
  include/admittance/Admittance.h
  src/admittance/Admittance.cpp
)
target_link_libraries(admittance_src
  ${catkin_LIBRARIES}
)

add_library(admittance_pos_src
  include/admittance_pos/Admittance_pos.h
  src/admittance_pos/Admittance_pos.cpp
)
target_link_libraries(admittance_pos_src
  ${catkin_LIBRARIES}
)


add_library(ur_kinematics_src
  include/ur_kin.h
  src/ur_kin.cpp
)
target_link_libraries(ur_kinematics_src
  ${catkin_LIBRARIES}
)

add_executable(ur5_control_test src/ur5_test_communication.cpp)
target_link_libraries(ur5_control_test ${catkin_LIBRARIES})

add_executable(gpt_demo src/arm_control_demo_gpt.cpp)
target_link_libraries(gpt_demo ${catkin_LIBRARIES})

add_executable(ur5_control src/ur5_control_node.cpp)
target_link_libraries(ur5_control ${catkin_LIBRARIES})

add_executable(test_kinematics src/test_Kinematics.cpp)
target_link_libraries(test_kinematics ${catkin_LIBRARIES})

add_executable(ur_kinematics src/ur_kin.cpp)
target_link_libraries(ur_kinematics ${catkin_LIBRARIES})

add_executable(admittance src/admittance/Admittance_node.cpp)
target_link_libraries(admittance ${catkin_LIBRARIES} admittance_src)

add_executable(admittance_pos src/admittance_pos/Admittance_pos_node.cpp)
target_link_libraries(admittance_pos ${catkin_LIBRARIES} admittance_pos_src ur_kinematics_src)