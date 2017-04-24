/**
ROS/IOP Bridge
Copyright (c) 2017 Fraunhofer

This program is dual licensed; you can redistribute it and/or
modify it under the terms of the GNU General Public License
version 2 as published by the Free Software Foundation, or
enter into a proprietary license agreement with the copyright
holder.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; or you can read the full license at
<http://www.gnu.de/documents/gpl-2.0.html>
*/

/** \author Alexander Tiderko */


#ifndef LOCALPOSESENSOR_RECEIVEFSM_H
#define LOCALPOSESENSOR_RECEIVEFSM_H

#include "JausUtils.h"
#include "InternalEvents/InternalEventHandler.h"
#include "Transport/JausTransport.h"
#include "JTSStateMachine.h"
#include "urn_jaus_jss_mobility_LocalPoseSensor_1_0/Messages/MessageSet.h"
#include "urn_jaus_jss_mobility_LocalPoseSensor_1_0/InternalEvents/InternalEventsSet.h"

typedef JTS::Receive_1_0 Receive;
typedef JTS::Send_1_0 Send;

#include "urn_jaus_jss_core_Transport_1_0/Transport_ReceiveFSM.h"
#include "urn_jaus_jss_core_Events_1_0/Events_ReceiveFSM.h"
#include "urn_jaus_jss_core_AccessControl_1_0/AccessControl_ReceiveFSM.h"

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <iop_builder_fkie/timestamp.h>


#include "LocalPoseSensor_ReceiveFSM_sm.h"

namespace urn_jaus_jss_mobility_LocalPoseSensor_1_0
{

class DllExport LocalPoseSensor_ReceiveFSM : public JTS::StateMachine
{
public:
	LocalPoseSensor_ReceiveFSM(urn_jaus_jss_core_Transport_1_0::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_Events_1_0::Events_ReceiveFSM* pEvents_ReceiveFSM, urn_jaus_jss_core_AccessControl_1_0::AccessControl_ReceiveFSM* pAccessControl_ReceiveFSM);
	virtual ~LocalPoseSensor_ReceiveFSM();

	/// Handle notifications on parent state changes
	virtual void setupNotifications();

	/// Action Methods
	virtual void SendAction(std::string arg0, Receive::Body::ReceiveRec transportData);
	virtual void updateLocalPoseAction(SetLocalPose msg);


	/// Guard Methods
	virtual bool isControllingClient(Receive::Body::ReceiveRec transportData);



	LocalPoseSensor_ReceiveFSMContext *context;

protected:

    /// References to parent FSMs
	urn_jaus_jss_core_Transport_1_0::Transport_ReceiveFSM* pTransport_ReceiveFSM;
	urn_jaus_jss_core_Events_1_0::Events_ReceiveFSM* pEvents_ReceiveFSM;
	urn_jaus_jss_core_AccessControl_1_0::AccessControl_ReceiveFSM* pAccessControl_ReceiveFSM;

	ros::NodeHandle p_nh;
	ros::Subscriber p_pose_sub;
	ros::Subscriber p_odom_sub;
	ReportLocalPose p_report_local_pose;
	std::string p_tf_frame_odom;
	std::string p_tf_frame_robot;
	ros::Timer p_tf_timer;
	tf::TransformListener tfListener;

	void tfCallback(const ros::TimerEvent& e);
	void poseReceived(const geometry_msgs::PoseStamped::ConstPtr& pose);
	void odomReceived(const nav_msgs::Odometry::ConstPtr& odom);
};

};

#endif // LOCALPOSESENSOR_RECEIVEFSM_H
