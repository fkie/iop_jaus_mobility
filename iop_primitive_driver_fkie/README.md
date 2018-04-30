This package is part of [ROS/IOP Bridge](https://github.com/fkie/iop_core/blob/master/README.md).


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


