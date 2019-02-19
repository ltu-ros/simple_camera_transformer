cmake_minimum_required(VERSION 2.8.3)
project(simple_camera_transformer)
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  dynamic_reconfigure
)

find_package(OpenCV REQUIRED)

generate_dynamic_reconfigure_options(cfg/Transformer.cfg)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES simple_camera_publisher
  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
  ${OpenCV_INCLUDE_DIRS}
  include
)

add_executable(cam_transformer src/cam_transformer.cpp)
add_dependencies(cam_transformer ${PROJECT_NAME}_gencfg)
target_link_libraries(cam_transformer ${catkin_LIBRARIES} ${OpenCV_LIBS})
