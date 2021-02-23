This package is part of [ROS/IOP Bridge](https://github.com/fkie/iop_core/blob/master/README.md).


## _fkie_iop_primitive_driver:_ PrimitiveDriver

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

#### Publisher:

_cmd_vel (geometry_msgs::TwistStamped)_

> Twist commands for the platform.

_cmd_vel (geometry_msgs::Twist)_

> Twist commands for the platform.

#### Subscriber:

> None


