cmake_minimum_required(VERSION 2.8.3)
project(bdf07_imu)

set(CMAKE_CXX_STANDARD 17)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
)

catkin_package(
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(bdf07_imu_node
        src/bdf07_imu_node.cpp
	 src/serial_utils.cpp 
)

target_link_libraries(bdf07_imu_node
  ${catkin_LIBRARIES}
)

install(
    TARGETS
        bdf07_imu_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS bdf07_imu_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY rviz
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz)


