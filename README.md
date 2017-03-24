# proto_grasp_movement

proto_grasp_movement is a code that lets to make grasp with a Pisa/IIT Soft Hand attached to a Kuka LWR. 

This packege needs of [ROS/indigo](http://wiki.ros.org/indigo/Installation/Ubuntu),[qb_interface_node](https://github.com/emalbt/qb_interface_node),[proto_grasp](https://github.com/redfulvio/proto_grasp),[CartesianImpedanceController](https://github.com/CentroEPiaggio/kuka-lwr/tree/master/lwr_controllers) on [Ubuntu 14.04](http://www.ubuntu.com/download/desktop).


## Use

The commands to launch the robot are:

`roslaunch proto_grasp_movement display_grasp_lauch`

`roslaunch proto_grasp_movement main.launch`

These codes to launch let to record data and recognize hits during a movement respectively:

`roslaunch qb_interface_node qb_interface.launch`

`roslaunch proto_grasp proto.launch`

## Notes

Some parameters can be changed: x_offset_, y_offset_, z_offset_ indicate the trajectory starting point; A_ is the trajectory amplitude; stiffness and torque to pass to the Cartesian Impedance Controller.


## TO DO

The commented part is referred to the possibilities to use a new data set to make grasp and to switch controllers to generate different trajectories.


rosservice call /right_arm/controller_manager/switch_controller "start_controllers: ['teleoperation_controller']
stop_controllers: ['joint_trajectory_controller']
strictness: 2" 


rostopic pub /right_arm/teleoperation_controller/start_controller std_msgs/Bool "data: true" 