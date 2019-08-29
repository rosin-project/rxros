cmake_minimum_required(VERSION 2.8.3)
project(rxros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rxcpp_vendor
  tf
)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    roscpp
    rxcpp_vendor
    tf
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
