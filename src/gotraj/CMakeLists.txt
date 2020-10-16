cmake_minimum_required(VERSION 2.8.3)
project(gotraj)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  trajectory_msgs
  geometry_msgs
  sensor_msgs
  tf
  tf_conversions
  eigen_conversions
  )


find_package(Boost REQUIRED system thread chrono regex log_setup log)


###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS   include
  LIBRARIES ${PROJECT_NAME} 
  CATKIN_DEPENDS
  roscpp
  moveit_core
  sensor_msgs
  trajectory_msgs
  geometry_msgs
  tf
  tf_conversions
  eigen_conversions
  DEPENDS
  Boost
  )

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  # include breakdown for this project
  #    include/${PROJECT_NAME}
  #   include/${PROJECT_NAME}/CrclXsd
  include/${PROJECT_NAME}/NIST
  /usr/include/eigen3
  )

set(CMAKE_CXX_FLAGS "-w ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-DDEBUG ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-DBOOST_LOG_DYN_LINK ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-fpermissive ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-g ${CMAKE_CXX_FLAGS}")

set(CMAKE_CXX_FLAGS "-DGO_REAL_DOUBLE ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-Wno-format-security ${CMAKE_CXX_FLAGS}")


set(CMAKE_EXE_LINKER_FLAGS "-rdynamic ${CMAKE_EXE_LINKER_FLAGS}")

set(GOTRAJ_LIB_SOURCES
  src/gointerp.cpp
  src/gomath.cpp
  src/gomotion.cpp
  src/gotraj.cpp
  src/gotraj_.cpp
  )    
add_library(gotraj ${GOTRAJ_LIB_SOURCES})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  /usr/lib/x86_64-linux-gnu/
  )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  )

install(TARGETS ${PROJECT_NAME} 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} #${CMAKE_INSTALL_PREFIX}
 RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)   

#install(FILES readme.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
