# ros_input_output

ROS nodes for streaming screen, keyboard and mouse events, and injecting keyboard and mouse commands as well.

## Building

catkin build ros_input_output

## Launching

roslaunch ros_input_output nodes.launch


## View screen

rosrun image_view image_view image:=/screen

## View keyboard/mouse events

rostopic echo /io/read

## Injection of keyboard or mouse commands 

This can be done my publishing gui_event messages (see description in msg/gui_event.msg)
to /io/write topic. For example

> rostopic pub /io/write     ros_input_output/gui_event "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
mouse_x: 0
mouse_y: 0
mouse_left: 0
mouse_right: 0
btncode: 0
type: 1"

will move mouse to top left corner.



