cmake_minimum_required(VERSION 2.8.3)
project(crcl_rosmsgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs 
  sensor_msgs
  )

add_message_files(
  FILES
  CrclCommandMsg.msg
  CrclMaxProfileMsg.msg
  CrclStatusMsg.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs  
  sensor_msgs 
)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs sensor_msgs)



