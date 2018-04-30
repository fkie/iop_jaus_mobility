This package is part of [ROS/IOP Bridge](https://github.com/fkie/iop_core/blob/master/README.md).


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
