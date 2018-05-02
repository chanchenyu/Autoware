cmake_minimum_required(VERSION 2.8.3)
project(utm)

find_package(catkin REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include/utm
  LIBRARIES utm
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include/utm
)

add_library(utm
  src/utm_pos_conv.cpp
)

#############
## Install ##
#############

install(
  DIRECTORY include/utm
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hh"
)

## Install executables and/or libraries
install(TARGETS utm
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
