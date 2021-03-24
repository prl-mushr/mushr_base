cmake_minimum_required(VERSION 3.0.2)
project(ackermann_cmd_mux)
set (CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  ackermann_msgs
  dynamic_reconfigure
  nodelet
  pluginlib
	roscpp
	rospy
  std_msgs
)

# pkg-config support
find_package(PkgConfig)
pkg_search_module(yaml-cpp REQUIRED yaml-cpp)

set(yaml-cpp_INCLUDE_DIRS ${yaml-cpp_INCLUDEDIR})

if(NOT "${yaml-cpp_VERSION}" VERSION_LESS "0.5")
  add_definitions(-DHAVE_NEW_YAMLCPP)
endif()

# Dynamic reconfigure support
generate_dynamic_reconfigure_options(cfg/reload.cfg)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}_nodelet
   CATKIN_DEPENDS ackermann_msgs dynamic_reconfigure nodelet pluginlib roscpp rospy std_msgs
   DEPENDS yaml-cpp
)

include_directories(include ${catkin_INCLUDE_DIRS} ${yaml-cpp_INCLUDE_DIRS})

# Nodelet library
add_library(${PROJECT_NAME}_nodelet src/ackermann_cmd_mux_nodelet.cpp src/ackermann_cmd_subscribers.cpp)
#add_dependencies(${PROJECT_NAME}_nodelet ackermann_msgs)
add_dependencies(${PROJECT_NAME}_nodelet ${PROJECT_NAME}_gencfg)

target_link_libraries(${PROJECT_NAME}_nodelet ${catkin_LIBRARIES} ${yaml-cpp_LIBRARIES})

install(TARGETS ${PROJECT_NAME}_nodelet
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY plugins
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY param
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
