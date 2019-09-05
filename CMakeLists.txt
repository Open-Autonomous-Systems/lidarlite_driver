cmake_minimum_required(VERSION 2.8.3)
project(lidarlite_driver)
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

set(PROJECT_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/include/" CACHE PATH "Project Include Directory")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  tf
  roslaunch
)
roslaunch_add_file_check(launch)

# Find Packages
find_package(Boost)
if(Boost_FOUND)
  find_package ( Boost COMPONENTS filesystem system serialization REQUIRED )
  set(Boost_GENERAL ${Boost_LIBRARIES})
elseif(NOT Boost_FOUND)
  message("Boost not found!")
endif()

catkin_python_setup()

#add_service_files(
#  FILES Basetasks.srv
#)

catkin_package(
  CATKIN_DEPENDS
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_INCLUDE_DIR}
  ${Boost_INCLUDE_DIR}
)

link_libraries(
  ${catkin_LIBRARIES}
)

set (LIBS ${Boost_GENERAL}) # ${ROS_LIBS} profiler
# Custom compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w -std=c++11")



add_executable (lidarlite_node 
src/lidarlite_driver_node.cpp
src/lidarlite_driver.cpp 
include/JHLidarLite_V2/src/lidarlite.cpp
)
#add_dependencies(lidarlite_node)
target_link_libraries(lidarlite_node ${catkin_LIBRARIES})

install(TARGETS lidarlite_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#catkin_install_python(PROGRAMS scripts/idt_demoSelector.py 
#scripts/make_squares_east_left.py scripts/mmwave_project.py 
#DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
