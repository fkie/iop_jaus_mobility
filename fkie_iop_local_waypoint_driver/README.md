This package is part of [ROS/IOP Bridge](https://github.com/fkie/iop_core/blob/master/README.md).


## _fkie_iop_local_waypoint_driver:_ LocalWaypointDriver

Forwards a local waypoint command to ROS.

#### Parameter:

_tf_frame_robot (str_, Default: "base_link")

> TF frame used in ROS for local coordinates. This value is set in each command message.

_tv_max (float_ , Default: 1.0)

> The maximum allowed speed.

#### Publisher:

_cmd_local_pose (geometry_msgs::PoseStamped)_

> The command position.

_cmd_travel_speed (std_msgs::msg::Float32)_, latched

> The maximum speed configured by parameter or send from OCU. Speed from OCU is always smaller or equal to parameterized value.

#### Subscriber:

_local_way_point_reached (std_msgs::msg::Bool)_

> Reports to the client service that the point is reached.

