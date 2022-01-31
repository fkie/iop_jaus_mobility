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
#include "urn_jaus_jss_mobility_LocalPoseSensor/Messages/MessageSet.h"
#include "urn_jaus_jss_mobility_LocalPoseSensor/InternalEvents/InternalEventsSet.h"

#include "InternalEvents/Receive.h"
#include "InternalEvents/Send.h"

#include "urn_jaus_jss_core_AccessControl/AccessControl_ReceiveFSM.h"
#include "urn_jaus_jss_core_Events/Events_ReceiveFSM.h"
#include "urn_jaus_jss_core_Transport/Transport_ReceiveFSM.h"


#include "LocalPoseSensor_ReceiveFSM_sm.h"
#include <rclcpp/rclcpp.hpp>
#include <fkie_iop_component/iop_component.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>


#include "LocalPoseSensor_ReceiveFSM_sm.h"

namespace urn_jaus_jss_mobility_LocalPoseSensor
{

class DllExport LocalPoseSensor_ReceiveFSM : public JTS::StateMachine
{
public:
	LocalPoseSensor_ReceiveFSM(std::shared_ptr<iop::Component> cmp, urn_jaus_jss_core_AccessControl::AccessControl_ReceiveFSM* pAccessControl_ReceiveFSM, urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM, urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM);
	virtual ~LocalPoseSensor_ReceiveFSM();

	/// Handle notifications on parent state changes
	virtual void setupNotifications();
	virtual void setupIopConfiguration();

	/// Action Methods
	virtual void SendAction(std::string arg0, Receive::Body::ReceiveRec transportData);
	virtual void updateLocalPoseAction(SetLocalPose msg);


	/// Guard Methods
	virtual bool isControllingClient(Receive::Body::ReceiveRec transportData);



	LocalPoseSensor_ReceiveFSMContext *context;

protected:

	/// References to parent FSMs
	urn_jaus_jss_core_AccessControl::AccessControl_ReceiveFSM* pAccessControl_ReceiveFSM;
	urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM;
	urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM;

	std::shared_ptr<iop::Component> cmp;
	rclcpp::Logger logger;
	iop::Timer p_tf_timer;
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr p_pose_sub;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr p_odom_sub;
	ReportLocalPose p_report_local_pose;
	std::string p_tf_frame_odom;
	std::string p_tf_frame_robot;
	std::unique_ptr<tf2_ros::Buffer> p_tf_buffer;
	std::shared_ptr<tf2_ros::TransformListener> p_tf_listener;

	void tfCallback();
	void poseReceived(const geometry_msgs::msg::PoseStamped::SharedPtr pose);
	void odomReceived(const nav_msgs::msg::Odometry::SharedPtr odom);
};

}

#endif // LOCALPOSESENSOR_RECEIVEFSM_H
