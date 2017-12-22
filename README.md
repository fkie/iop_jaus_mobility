See [iop_core](https://github.com/fkie/iop_core/blob/master/README.md) for use instructions.

# Interfaces

List of service plugins in this repository:

[iop_global_pose_sensor_fkie: GlobalPoseSensor](#iop_global_pose_sensor_fkie-globalposesensor)  
[iop_global_waypoint_driver_fkie: GlobalWaypointDriver](#iop_global_waypoint_driver_fkie-globalwaypointdriver)  
[iop_global_waypoint_list_driver_fkie: GlobalWaypointListDriver](#iop_global_waypoint_list_driver_fkie-globalwaypointlistdriver)  
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

_cmd_global_pose (geometry_msgs::PoseStamped)_

> The command position.

_cmd_travel_speed (std_msgs::Float32)_, latched

> The maximum speed configured by parameter or send from OCU. Speed from OCU is always smaller or equal to parameterized value.

#### Subscriber:

_global_way_point_reached (std_msgs::Bool)_

> Reports to the client service that the point is reached.


## _iop_global_waypoint_list_driver_fkie:_ GlobalWaypointListDriver

Forwards the list of global waypoints to ROS.

#### Parameter:

_tf_frame_world (str_, Default: "/world")

> TF frame used in ROS for global coordinates. This value is set in each command message.

_tv_max (float_ , Default: 1.0)

> The maximum allowed speed.

#### Publisher:

_cmd_global_waypoints (nav_msgs::Path)_

> The list with global waypoints. It can also be an empty list to abort the execution.

_cmd_travel_speed (std_msgs::Float32)_, latched

> The maximum speed configured by parameter or send from OCU. Speed from OCU is always smaller or equal to parameterized value.

#### Subscriber:

_global_way_points_finished (std_msgs::Bool)_

> Reports to the client service that the excution is finished.


## _iop_local_pose_sensor_fkie:_ LocalPoseSensor

The local position can be obtained from three different sources: ```Tf```, ```geometry_msgs::PoseStamped``` or ```nav_msgs::Odometry```

#### Parameter:

_source_type (int_, Default: 0)

> Defines the source of local position. 0: tf, 1: geometry_msgs::PoseStamped, 2: nav_msgs::Odometry

_tf_frame_odom (str_, Default: "odom")

> Defines the odometry frame id. This parameter is only regarded if _source_type_ is *0*.

_tf_frame_robot (str_, Default: "base_link")

> Defines the robot frame id. This parameter is only regarded if _source_type_ is *0*.

#### Publisher:

> None

#### Subscriber:

_pose (geometry_msgs::PoseStamped)_

> Reads the local position. This subscriber is only valid if _source_type_ is *1*.

_odom (nav_msgs::Odometry)_

> Reads the local position. This subscriber is only valid if _source_type_ is *2*.


## _iop_primitive_driver_fkie:_ PrimitiveDriver

Sends the IOP effort command messages as ROS Twist message to a robot platform.

#### Parameter:

_max_linear_x (double_, Default: 3.5)

> The maximum forward velocity allowed in meters/sec. Negative value inverts the direction.

_max_linear_y (double_, Default: 0)

> The maximum y velocity allowed in meters/sec. Negative value inverts the direction.

_max_linear_z (double_, Default: 0)

> The maximum z velocity allowed in meters/sec. Negative value inverts the direction.

_max_angular_x (double_, Default: 0)

> The maximum roll rotation velocity allowed in radians/sec. Negative value inverts the direction.

_max_angular_y (double_, Default: 0)

> The maximum pitch rotation velocity allowed in radians/sec. Negative value inverts the direction.

_max_angular_z (double_, Default: 1.5)

> The maximum yaw rotation velocity allowed in radians/sec. Negative value inverts the direction.

_use_stamped (bool_, Default: true)

> If *true* use _geometry_msgs::TwistStamped_ instead of _geometry_msgs::Twist_ to publish the commands.

#### Publisher:

_cmd_vel (geometry_msgs::TwistStamped)_

> Twist commands for the platform, if _use_stamped_ is set to *true*.

_cmd_vel (geometry_msgs::Twist)_

> Twist commands for the platform, if _use_stamped_ is set to *false*.

#### Subscriber:

> None


