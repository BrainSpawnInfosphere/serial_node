cmake_minimum_required(VERSION 2.8.3)
project(ip_serial)

###
# catkin_make -DCMAKE_INSTALL_PREFIX=/Users/kevin/catkin/install install
###

## Find catkin and any catkin packages on which
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  rospy 
  genmsg 
  std_msgs 
  message_generation
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Generate services in the 'srv' folder
add_service_files(
   FILES 
      serial.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES 
      std_msgs
)

## Declare things to be passed to other projects
catkin_package(
  #INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs message_runtime
#  DEPENDS Boost # other-3rd-party-libraries
)

## Specify additional locations of header files
include_directories(${catkin_INCLUDE_DIRS} ) #${Boost_INCLUDE_DIRS})

## Declare a cpp executable
add_executable(ip_serial src/serial_node_service.cpp)
target_link_libraries(ip_serial ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp) # this will build the msgs/srv

message("Look at me:  ${CATKIN_PACKAGE_BIN_DESTINATION} =")

## Mark executables and/or libraries for installation
 install(TARGETS ${PROJECT_NAME}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark cpp header files for installation
 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_test3.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add any nosetest based python tests
# catkin_add_nosetests(test)
