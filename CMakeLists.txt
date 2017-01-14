cmake_minimum_required(VERSION 2.8.3)
project(mantis)


find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  dynamic_reconfigure
  cmake_modules
  tf
)

find_package(OpenCV REQUIRED)

include_directories(
	${OpenCV_INCLUDE_DIRS}
	include
	include/mantis
	${catkin_INCLUDE_DIRS}
)

catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES pauvsi_vio
# CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs tf
#  DEPENDS system_lib
   DEPENDS opencv
)

add_library(montecarlo include/mantis/MonteCarlo.cpp)

add_executable(mantis_client src/mantis_client.cpp)
add_executable(mantis_server src/mantis_server.cpp)

target_link_libraries(montecarlo ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(mantis_client ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(mantis_server ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} montecarlo)