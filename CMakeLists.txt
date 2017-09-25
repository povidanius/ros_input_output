cmake_minimum_required(VERSION 2.8.3)
project(ros_input_output)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs std_msgs geometry_msgs
  image_transport cv_bridge
  genmsg message_runtime message_generation
)

find_package(OpenCV REQUIRED)

 add_message_files(
   FILES
   gui_event.msg  
 )

# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

 generate_messages(
   DEPENDENCIES
   sensor_msgs std_msgs geometry_msgs
 )

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES keystack_lib
#  CATKIN_DEPENDS roscpp sensor_msgs std_msgs
  DEPENDS system_lib message_runtime
)

include_directories(
  "include"
  ${catkin_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)



add_executable(screen_node src/screengrab_ros_node.cpp)

target_link_libraries(screen_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
   X11
)

add_executable(keyboard_node src/keyboard_ros_node.cpp)
target_link_libraries(keyboard_node
   X11
   xosd
   pthread
   Xext
   Xinerama
   Xtst
   ${catkin_LIBRARIES}
   ${roscpp_LIBRARIES}
)


add_executable(mouse_node src/mouse_ros_node.cpp)
add_dependencies(mouse_node ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(mouse_node
   X11
   xosd
   pthread
   Xext
   Xinerama
   Xtst
   ${catkin_LIBRARIES}
   ${roscpp_LIBRARIES}
)



