cmake_minimum_required(VERSION 2.8.3)
project(mantis)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  geometry_msgs
  std_msgs
  dynamic_reconfigure
  cmake_modules
  tf
  message_generation
)

find_package(OpenCV REQUIRED)

find_package(Eigen3 REQUIRED)

include_directories(
	${OpenCV_INCLUDE_DIRS}
	include
	include/mantis
	${catkin_INCLUDE_DIRS}
	
	${Eigen_INCLUDE_DIRS}
)


# add the service files
add_service_files(
	FILES
	mantisService.srv
)

generate_messages(
	DEPENDENCIES
	std_msgs
	geometry_msgs
	sensor_msgs
)

catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES pauvsi_vio
# CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs tf
#  DEPENDS system_lib
   DEPENDS opencv
)

add_executable(mantis_test test/unit_test.cpp)
target_link_libraries(mantis_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES})


add_executable(mantis2 src/mantis2.cpp)
target_link_libraries(mantis2 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES})

add_library(montecarlo include/mantis/MonteCarlo.cpp)
set_target_properties(montecarlo PROPERTIES LINKER_LANGUAGE CXX)

add_executable(mantis_client src/mantis_client.cpp)
add_executable(mantis_server src/mantis_server.cpp)

target_link_libraries(montecarlo ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(mantis_client ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES})
target_link_libraries(mantis_server ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES} montecarlo)

add_dependencies(mantis_client ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(mantis_server ${PROJECT_NAME}_generate_messages_cpp)