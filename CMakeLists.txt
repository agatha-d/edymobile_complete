cmake_minimum_required(VERSION 3.0.2)
project(edymobile_complete)

find_package(catkin REQUIRED)
catkin_package()

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()

install(DIRECTORY images meshes launch rviz urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

##catkin_install_python(PROGRAMS src/sendGoals_multiple.py
##   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
## )
 
 ## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
 catkin_install_python(PROGRAMS src/generate_goals.py
#   scripts/my_python_script
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
