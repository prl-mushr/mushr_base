cmake_minimum_required(VERSION 2.8.3)
project(mushr_base)

find_package(catkin REQUIRED)


catkin_python_setup()

catkin_package()


catkin_install_python(PROGRAMS scripts/racecar_state scripts/joy_teleop
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
