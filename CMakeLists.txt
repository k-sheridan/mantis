cmake_minimum_required(VERSION 2.8.3)
project(mantis)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  tf
  tf2_ros
  cmake_modules
  std_msgs
)

find_package(OpenCV REQUIRED)

include_directories(
	${OpenCV_INCLUDE_DIRS}
	include
	${catkin_INCLUDE_DIRS}
)

catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES pauvsi_vio
#  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
   DEPENDS opencv
)

add_library(montecarlo include/mantis/MonteCarlo.cpp)

add_executable(mantis_client src/mantis_client.cpp)

target_link_libraries(mantis_client ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})