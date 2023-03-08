cmake_minimum_required(VERSION 3.1)
project(robot_msgs)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR})

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  actionlib
  message_generation
)

# add_action_files(
#   DIRECTORY action
#   FILES
# )

# add_message_files(
#   DIRECTORY msg
#   FILES
#   IMUParam.msg
#   TOFdata.msg
# )

# add_message_files(
#   DIRECTORY msg/net
#   FILES
#   ZMQdata.msg
# )

add_message_files(
  DIRECTORY msg/robot
  FILES
  OdomGoalMsg.msg
)

add_message_files(
  DIRECTORY msg/sensors
  FILES
  AutoAimSensorsMsg.msg
  AutoAimSensorsParam.msg
)

# add_message_files(
#   DIRECTORY msg/referee
#   FILES
#   GameZone.msg
#   GameBuff.msg
#   GameResult.msg
#   GameRobotHP.msg
#   GameStatus.msg
#   RobotDamage.msg
#   RobotStatus.msg
# )

# add_message_files(
#   DIRECTORY msg/wireless
#   FILES
#   SentryData.msg
#   SharePath.msg
#   TeamInfo.msg
#   WirelessParam.msg
# )

# add_service_files(
#   DIRECTORY srv
#   FILES
# )

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
)
