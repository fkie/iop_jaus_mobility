This package is part of [ROS/IOP Bridge](https://github.com/fkie/iop_core/blob/master/README.md).


## _fkie_iop_global_waypoint_driver:_ GlobalWaypointDriver

Forwards the global waypoint command to ROS.

#### Parameter:

_tf_frame_world (str_, Default: "/world")

> TF frame used in ROS for global coordinates. This value is set in each command message.

_tv_max (float_ , Default: 1.0)

> The maximum allowed speed.

#### Publisher:

_cmd_global_pose (geometry_msgs::PoseStamped)_

> The command position.

_cmd_fix (sensor_msgs::NavSatFix)_

> Alternative command position without orientation.

_cmd_global_geopose (geographic_msgs::GeoPoseStamped)_

> Alternative command position.

_cmd_travel_speed (std_msgs::Float32)_, latched

> The maximum speed configured by parameter or send from OCU. Speed from OCU is always smaller or equal to parameterized value.

#### Subscriber:

_global_way_point_reached (std_msgs::Bool)_

> Reports to the client service that the point is reached.

