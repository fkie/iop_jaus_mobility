This package is part of [ROS/IOP Bridge](https://github.com/fkie/iop_core/blob/master/README.md).


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

