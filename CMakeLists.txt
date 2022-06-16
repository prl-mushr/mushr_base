cmake_minimum_required(VERSION 3.0.2)
project(mushr_base)

find_package(catkin REQUIRED)


catkin_python_setup()

catkin_package()


catkin_install_python(PROGRAMS scripts/racecar_state scripts/joy_teleop scripts/nav_msg_converter
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY config launch maps
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})