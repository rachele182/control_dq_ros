cmake_minimum_required(VERSION 3.0.2)
project(panda_controllers)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Cose che ho aggiunto contrassegnate con ##
find_package(catkin REQUIRED COMPONENTS
  controller_interface
  dynamic_reconfigure
  franka_hw
  geometry_msgs
  franka_msgs 
  hardware_interface
  tf
  tf_conversions
  message_generation
  pluginlib
  realtime_tools
  roscpp
  rospy
) 

find_package(Eigen3 REQUIRED)
find_package(ALGLIB REQUIRED)
find_package(Franka 0.9.0 REQUIRED)

add_message_files(FILES
  RobotState.msg
  DesiredProjectTrajectory.msg
  EEpose.msg
  InfoDebug.msg
  CompliantTraj.msg
  DesiredImpedance.msg
)

##
generate_messages(
  DEPENDENCIES##
  franka_msgs##
  std_msgs
  geometry_msgs
)

generate_dynamic_reconfigure_options(
  cfg/compliance_param.cfg
  cfg/desired_mass_param.cfg
)
   
##
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    controller_interface
    dynamic_reconfigure
    franka_hw
    geometry_msgs
    franka_msgs##
    hardware_interface
    tf
    tf_conversions
    message_runtime
    pluginlib
    realtime_tools
    roscpp
  DEPENDS Franka
)

add_library(${PROJECT_NAME}
  src/motion_control_dq.cpp #controller 
  src/impedance_control_dq.cpp 
  src/dual_arm_control.cpp #dual arm controller 
  #src/dq_trajectory.cpp #trajectory
  # src/impedance_loop.cpp
  # src/commands.cpp
  src/Dynamics.cpp
  # src/CoriolisMatrix.cpp
  src/parsing_utilities.cpp #util for yaml parameters
) 

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  panda_controllers ${catkin_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  ${PROJECT_NAME}_gencpp
  ${PROJECT_NAME}_gencfg
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  ${Franka_LIBRARIES}
  ${catkin_LIBRARIES}
  dqrobotics
)

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  ${Franka_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  include
)

include_directories(${catkin_INCLUDE_DIRS})

include_directories(${EIGEN3_INCLUDE_DIRS})

 
include_directories(
   include
 )
 
add_executable(commands
  src/commands.cpp
)

# add_executable(modulator
#   src/modulator.cpp
# )

add_executable(modulator_2
  src/modulator_2.cpp
)

# add_executable(dq_trajectory
#    src/dq_trajectory.cpp
#  )

add_executable(impedance_loop
  src/impedance_loop.cpp
) 
#dual-arm impedance loop
add_executable(dual_impedance_loop
  src/dual_impedance_loop.cpp
) 

target_link_libraries(commands PUBLIC
  ${Franka_LIBRARIES}
  ${catkin_LIBRARIES}
  dqrobotics
)

# target_link_libraries(modulator PUBLIC
#   ${Franka_LIBRARIES}
#   ${catkin_LIBRARIES}
#   dqrobotics
# )

target_link_libraries(modulator_2 PUBLIC
  ${Franka_LIBRARIES}
  ${catkin_LIBRARIES}
  dqrobotics
)

# target_link_libraries(dq_trajectory PUBLIC
#   ${Franka_LIBRARIES}
#   ${catkin_LIBRARIES}
#   dqrobotics
# )

target_link_libraries(impedance_loop PUBLIC
  ${Franka_LIBRARIES}
  ${catkin_LIBRARIES}
  dqrobotics
)

target_link_libraries(dual_impedance_loop PUBLIC
  ${Franka_LIBRARIES}
  ${catkin_LIBRARIES}
  dqrobotics
)

# target_link_libraries(impedance_loop ${PROJECT_NAME})
# target_link_libraries(commands ${PROJECT_NAME})
# target_link_libraries(dq_trajectory ${PROJECT_NAME})

target_link_libraries(${PROJECT_NAME} PUBLIC
  ${Franka_LIBRARIES}
  ${catkin_LIBRARIES}
)

# target_include_directories(dq_trajectory PUBLIC
#   ${catkin_INCLUDE_DIRS}
# )
# target_link_libraries(dq_trajectory ${PROJECT_NAME}
#   ${Franka_LIBRARIES}
#   ${catkin_LIBRARIES}
#   dqrobotics
#   dqrobotics-interface-vrep)

# add_dependencies(commands
#   ${${PROJECT_NAME}_EXPORTED_TARGETS}
#   ${catkin_EXPORTED_TARGETS}
#   ${PROJECT_NAME}_generate_messages_cpp
#   ${PROJECT_NAME}_gencpp
#   ${PROJECT_NAME}_gencfg
# )

## Installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES ${PROJECT_NAME}_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

