cmake_minimum_required(VERSION 2.8.3)
project(rmui_drivers)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

install(DIRECTORY config launch rvizconfig
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

if(CATKIN_ENABLE_TESTING)
  find_package(roslint REQUIRED)
  find_package(rostest REQUIRED)
  set(ROSLINT_PYTHON_OPTS "--max-line-length=80")
  roslint_python()
endif()
