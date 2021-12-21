cmake_minimum_required(VERSION 2.8.3)
project(lidarlite_driver)
add_compile_options(-std=c++11)
########################################################
# https://stackoverflow.com/a/63723420
function(get_linux_lsb_release_information)
  find_program(LSB_RELEASE_EXEC lsb_release)
  if(NOT LSB_RELEASE_EXEC)
    message(FATAL_ERROR "Could not detect lsb_release executable, can not gather required information")
  endif()

  execute_process(COMMAND "${LSB_RELEASE_EXEC}" --short --id OUTPUT_VARIABLE LSB_RELEASE_ID_SHORT OUTPUT_STRIP_TRAILING_WHITESPACE)
  execute_process(COMMAND "${LSB_RELEASE_EXEC}" --short --release OUTPUT_VARIABLE LSB_RELEASE_VERSION_SHORT OUTPUT_STRIP_TRAILING_WHITESPACE)
  execute_process(COMMAND "${LSB_RELEASE_EXEC}" --short --codename OUTPUT_VARIABLE LSB_RELEASE_CODENAME_SHORT OUTPUT_STRIP_TRAILING_WHITESPACE)

  set(LSB_RELEASE_ID_SHORT "${LSB_RELEASE_ID_SHORT}" PARENT_SCOPE)
  set(LSB_RELEASE_VERSION_SHORT "${LSB_RELEASE_VERSION_SHORT}" PARENT_SCOPE)
  set(LSB_RELEASE_CODENAME_SHORT "${LSB_RELEASE_CODENAME_SHORT}" PARENT_SCOPE)
endfunction()

if(CMAKE_SYSTEM_NAME MATCHES "Linux")
  get_linux_lsb_release_information()
  if(${LSB_RELEASE_VERSION_SHORT} STREQUAL "16.04")
    message("detected ubuntu 16")
    set(i2c_from_linux "0")
  else()
    message("not ubuntu 16, include i2c headers from kernel")
    set(i2c_from_linux "1")
  endif()
endif()
add_definitions(-Di2c_from_linux_${i2c_from_linux})
#############################################################################3
set(PROJECT_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/include/" CACHE PATH "Project Include Directory")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nodelet
  #tf
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
#catkin_python_setup()

#add_service_files(
#  FILES Basetasks.srv
#)

catkin_package(
  CATKIN_DEPENDS
  nodelet
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_INCLUDE_DIR}
  ${Boost_INCLUDE_DIR}
)

add_library(lidarlite_driver_lib src/lidarlite_driver.cpp
        include/JHLidarLite_V2/src/lidarlite.cpp
        include/JHLidarLite_V2/src/lidarlite.h
        include/fir_filter/fir_filter.h)

if(${i2c_from_linux} STREQUAL 1)
  set(ADDITIONAL_LIBRARIES i2c)
else()
  set(ADDITIONAL_LIBRARIES "")
endif()

target_link_libraries(lidarlite_driver_lib
        ${catkin_LIBRARIES}
        ${ADDITIONAL_LIBRARIES})

link_libraries(
  ${catkin_LIBRARIES}
)

set (LIBS ${Boost_GENERAL})
# Custom compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w -std=c++11")

add_executable (lidarlite_node src/lidarlite_driver_node.cpp)
target_link_libraries(lidarlite_node ${catkin_LIBRARIES})

install(TARGETS lidarlite_node lidarlite_driver_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
