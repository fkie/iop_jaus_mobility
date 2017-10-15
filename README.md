See [iop_core](https://github.com/fkie/iop_core/blob/master/README.md) for use instructions.

# Interfaces

List of service plugins in this repository:

[iop_global_pose_sensor_fkie: GlobalPoseSensor](#iop_global_pose_sensor_fkie-globalposesensor)  
[iop_global_waypoint_driver_fkie: GlobalWaypointDriver](#iop_global_waypoint_driver_fkie-globalwaypointdriver)  
[iop_local_pose_sensor_fkie: LocalPoseSensor](#iop_local_pose_sensor_fkie-localposesensor)  
[iop_primitive_driver_fkie: PrimitiveDriver](#iop_primitive_driver_fkie-primitivedriver)

## _iop_global_pose_sensor_fkie:_ GlobalPoseSensor

Subscribes NavSatFix and Imu ROS messages and converts these into ReportGlobalPose. 

#### Parameter:

> None

#### Publisher:

> None

#### Subscriber:

_fix (sensor_msgs::NavSatFix)_

> If global position is received a ReportGlobalPose is created. Uses a previously received global orientation.

_imu (sensor_msgs::Imu)_

> Store global orientation and create ReportGlobalPose only if global position is received.

## _iop_global_waypoint_driver_fkie:_ GlobalWaypointDriver

Forwards the global waypoint command to ROS.

#### Parameter:

_tf_frame_world (str_, Default: "/world")

> TF frame used in ROS for global coordinates. This value is set in each command message.

_tv_max (float_ , Default: 1.0)

> The maximum allowed speed.

#### Publisher:

_cmd_global_waypoint (nav_msgs::Path)_

> The path contains always one point.

_cmd_tv_max (std_msgs::Float32)_, latched

> The maximum speed configured by parameter or send from OCU. Speed from OCU is always smaller or equal to parameterized value.

#### Subscriber:

> None

## _iop_local_pose_sensor_fkie:_ LocalPoseSensor

The local position can be obtained from three different sources: ```Tf```, ```geometry_msgs::PoseStamped``` or ```nav_msgs::Odometry```

# To be continued ...

