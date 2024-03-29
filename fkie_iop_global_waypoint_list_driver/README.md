This package is part of [ROS/IOP Bridge](https://github.com/fkie/iop_core/blob/master/README.md).


## _fkie_iop_global_waypoint_list_driver:_ GlobalWaypointListDriver

Forwards the list of global waypoints to ROS.

#### Parameter:

_tf_frame_world (str_, Default: "/world")

> TF frame used in ROS for global coordinates. This value is set in each command message.

_tv_max (float_ , Default: 1.0)

> The maximum allowed speed.

#### Publisher:

_cmd_global_waypoints (nav_msgs::Path)_

> The list with global waypoints. It can also be an empty list to abort the execution.

_cmd_global_geopath (geographic_msgs::GeoPath)_

> The list with global waypoints. It can also be an empty list to abort the execution.

_cmd_travel_speed (std_msgs::Float32)_, latched

> The maximum speed configured by parameter or send from OCU. Speed from OCU is always smaller or equal to parameterized value.

_cmd_global_execute (std_msgs::Bool)_

> Sends True after waypoints are published.

#### Subscriber:

_global_way_points_finished (std_msgs::Bool)_

> Reports to the client service that the excution is finished.

