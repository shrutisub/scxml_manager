cmake_minimum_required(VERSION 2.8.3)
project(scxml_manager)

find_package(catkin REQUIRED COMPONENTS
  rospy
  scxml_interpreter
  qt_smach_viewer
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy scxml_interpreter qt_smach_viewer
)



install(PROGRAMS
   scripts/scxml_interpreter_node
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
   launch/scxml_interpreter.launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY resources
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
