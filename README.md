# MuSHR Base
MuSHR repo for base packages necessary for running the car in sim and on the robot.

### Mushr_base API
Joystick params can be set in mushr_base/config/joy_teleop.yaml

#### Publishers
`/car_pose` [geometry_msgs/PoseStamped] Pose of robot. Published when map --> base_footprint transforms exist (sim).
`/joint_states` [sensor_msgs/JointState] Joint states from robot model and position
`/mux/ackermann_cmd_mux/input/teleop` [ackermann_msgs/AckermannDriveStamped] publish teleop controls from either keyboard (sim) or joystick (real robot)
`/tf` [tf2_msgs/TFMessage] all transforms

#### Subscribers
`/initialpose` [geometry_msgs/PoseWithCovarianceStamped] Initial pose of robot, usually provided from rviz.  
`/vesc/sensors/core` [vesc_msgs/VescStateStamped] vesc state. Speed param used to get controls.  
`/vesc/sensors/servo_position_command` [std_msgs/Float64] steering servo state

### Ackermann_cmd_mux API
Parameters can be set in ackermann_cmd_mux/param/mux.yaml

#### Subscribers
`/mux/ackermann_cmd_mux/input/default` [ackermann_msgs/AckermannDriveStamped] default input to car if not input control  
`/mux/ackermann_cmd_mux/input/navigation` [ackermann_msgs/AckermannDriveStamped] controller's input channel to drive car  
`/mux/ackermann_cmd_mux/input/safety` [ackermann_msgs/AckermannDriveStamped] safety controller's input channel. Currently null 
`/mux/ackermann_cmd_mux/input/teleop` [ackermann_msgs/AckermannDriveStamped] teleop controller's input channel  

#### Publishers
`/mux/ackermann_cmd_mux/output` [ackermann_msgs/AckermannDriveStamped] output of muxed inputs topics
`/mux/ackermann_cmd_mux/active` [std_msgs/String] which input is the current output 
