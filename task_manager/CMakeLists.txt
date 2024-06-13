cmake_minimum_required(VERSION 3.0.2)
project(task_manager)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  behaviortree_cpp_v3
  actionlib
  actionlib_msgs
  utbots_face_recognition
)


catkin_package(
  CATKIN_DEPENDS 
  roscpp 
  behaviortree_cpp_v3 
  utbots_face_recognition
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

set(BEHAVIOR_TREE_LIBRARY "behaviortree_cpp_v3::behaviortree_cpp_v3")

add_executable(recognitionMain bt_src/recognitionMain.cpp bt_src/recognition.cpp)

add_dependencies(recognitionMain ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(recognitionMain ${catkin_LIBRARIES} )

add_dependencies(recognitionMain ${utbots_face_recognition_EXPORTED_TARGETS})

catkin_install_python(PROGRAMS src/coordinate_tasks.py src/personal_recognition.py src/smach_example.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
