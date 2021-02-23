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


#include "urn_jaus_jss_mobility_LocalPoseSensor/LocalPoseSensor_ReceiveFSM.h"
#include <fkie_iop_component/iop_config.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>


using namespace JTS;

namespace urn_jaus_jss_mobility_LocalPoseSensor
{



LocalPoseSensor_ReceiveFSM::LocalPoseSensor_ReceiveFSM(std::shared_ptr<iop::Component> cmp, urn_jaus_jss_core_AccessControl::AccessControl_ReceiveFSM* pAccessControl_ReceiveFSM, urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM, urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM)
: logger(cmp->get_logger().get_child("LocalPoseSensor")),
  p_tf_timer(std::chrono::seconds(1), std::bind(&LocalPoseSensor_ReceiveFSM::tfCallback, this), false)
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new LocalPoseSensor_ReceiveFSMContext(*this);

	this->pAccessControl_ReceiveFSM = pAccessControl_ReceiveFSM;
	this->pEvents_ReceiveFSM = pEvents_ReceiveFSM;
	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	this->cmp = cmp;
	p_tf_frame_odom = "odom";
	p_tf_frame_robot = "base_link";
}



LocalPoseSensor_ReceiveFSM::~LocalPoseSensor_ReceiveFSM()
{
	delete context;
}

void LocalPoseSensor_ReceiveFSM::setupNotifications()
{
	pAccessControl_ReceiveFSM->registerNotification("Receiving_Ready_NotControlled", ieHandler, "InternalStateChange_To_LocalPoseSensor_ReceiveFSM_Receiving_Ready_NotControlled", "AccessControl_ReceiveFSM");
	pAccessControl_ReceiveFSM->registerNotification("Receiving_Ready_Controlled", ieHandler, "InternalStateChange_To_LocalPoseSensor_ReceiveFSM_Receiving_Ready_Controlled", "AccessControl_ReceiveFSM");
	pAccessControl_ReceiveFSM->registerNotification("Receiving_Ready", ieHandler, "InternalStateChange_To_LocalPoseSensor_ReceiveFSM_Receiving_Ready_NotControlled", "AccessControl_ReceiveFSM");
	pAccessControl_ReceiveFSM->registerNotification("Receiving", ieHandler, "InternalStateChange_To_LocalPoseSensor_ReceiveFSM_Receiving_Ready_NotControlled", "AccessControl_ReceiveFSM");
	registerNotification("Receiving_Ready_NotControlled", pAccessControl_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControl_ReceiveFSM_Receiving_Ready_NotControlled", "LocalPoseSensor_ReceiveFSM");
	registerNotification("Receiving_Ready_Controlled", pAccessControl_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControl_ReceiveFSM_Receiving_Ready_Controlled", "LocalPoseSensor_ReceiveFSM");
	registerNotification("Receiving_Ready", pAccessControl_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControl_ReceiveFSM_Receiving_Ready", "LocalPoseSensor_ReceiveFSM");
	registerNotification("Receiving", pAccessControl_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControl_ReceiveFSM_Receiving", "LocalPoseSensor_ReceiveFSM");
}


void LocalPoseSensor_ReceiveFSM::setupIopConfiguration()
{
	iop::Config cfg(cmp, "LocalPoseSensor");
	int source = 0;
	cfg.declare_param<uint8_t>("tv_max", source, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER,
		"Defines the source of local position. 0: tf, 1: geometry_msgs::PoseStamped, 2: nav_msgs::Odometry",
		"Default: 0");
	cfg.declare_param<std::string>("tf_frame_odom", p_tf_frame_odom, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_STRING,
		"Defines the odometry frame id. This parameter is only regarded if _source_type_ is *0* (tf).",
		"Default: 'odom'");
	cfg.declare_param<std::string>("p_tf_frame_robot", p_tf_frame_robot, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_STRING,
		"Defines the robot frame id. This parameter is only regarded if _source_type_ is *0* (tf).",
		"Default: 'base_link'");
	std::map<int, std::string> source_map;
	source_map[0] = "tf";
	source_map[1] = "PoseStamped";
	source_map[2] = "Odometry";
	cfg.param_named("source_type", source, source, source_map, true, "");
	switch (source) {
		case 1 :
			p_pose_sub = cfg.create_subscription<geometry_msgs::msg::PoseStamped>("pose", 1, std::bind(&LocalPoseSensor_ReceiveFSM::poseReceived, this, std::placeholders::_1));
			break;
		case 2 :
			p_odom_sub = cfg.create_subscription<nav_msgs::msg::Odometry>("odom", 1, std::bind(&LocalPoseSensor_ReceiveFSM::odomReceived, this, std::placeholders::_1));
			break;
		default :
			cfg.param("tf_frame_odom", p_tf_frame_odom, p_tf_frame_odom);
			cfg.param("tf_frame_robot", p_tf_frame_robot, p_tf_frame_robot);
			p_tf_buffer = std::make_unique<tf2_ros::Buffer>(cmp->get_clock());
			p_tf_listener = std::make_shared<tf2_ros::TransformListener>(*p_tf_buffer);
			double tf_hz = 10.0;
			cfg.param("tf_hz", tf_hz, tf_hz);
			if (tf_hz == 0.0) {
				tf_hz = 10.0;
			}
			p_tf_timer.set_rate(tf_hz);
			p_tf_timer.start();
			break;
	}
	pEvents_ReceiveFSM->get_event_handler().register_query(QueryLocalPose::ID);
}

void LocalPoseSensor_ReceiveFSM::SendAction(std::string arg0, Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
	/// Insert User Code HERE
	RCLCPP_DEBUG(logger, "LocalPoseSensor", "request %s from %d.%d.%d", arg0.c_str(),
			  transportData.getSrcSubsystemID(), transportData.getSrcNodeID(), transportData.getSrcComponentID());
	JausAddress sender = JausAddress(transportData.getSrcSubsystemID(),
									 transportData.getSrcNodeID(),
									 transportData.getSrcComponentID());
	if (strcmp(arg0.c_str(), "ReportLocalPose") == 0) {
	  this->sendJausMessage(p_report_local_pose, sender);
	}
}

void LocalPoseSensor_ReceiveFSM::updateLocalPoseAction(SetLocalPose msg)
{
	/// Insert User Code HERE
	RCLCPP_WARN(logger, "LocalPoseSensor", "updateLocalPoseAction not implemented");
}



bool LocalPoseSensor_ReceiveFSM::isControllingClient(Receive::Body::ReceiveRec transportData)
{
	//// By default, inherited guards call the parent function.
	//// This can be replaced or modified as needed.
	return pAccessControl_ReceiveFSM->isControllingClient(transportData );
}

void LocalPoseSensor_ReceiveFSM::tfCallback()
{
	try {
		geometry_msgs::msg::TransformStamped tfs = p_tf_buffer->lookupTransform(p_tf_frame_odom, p_tf_frame_robot, tf2::TimePointZero);
		p_report_local_pose.getBody()->getLocalPoseRec()->setX(tfs.transform.translation.x);
		p_report_local_pose.getBody()->getLocalPoseRec()->setY(tfs.transform.translation.y);
		p_report_local_pose.getBody()->getLocalPoseRec()->setZ(tfs.transform.translation.z);
		try {
			tf2::Quaternion quat(tfs.transform.rotation.x, tfs.transform.rotation.y,
					tfs.transform.rotation.z, tfs.transform.rotation.w);

			tf2::Matrix3x3 m(quat);
			double yaw, pitch, roll;
			m.getRPY(roll, pitch, yaw);
			p_report_local_pose.getBody()->getLocalPoseRec()->setRoll(roll);
			p_report_local_pose.getBody()->getLocalPoseRec()->setPitch(pitch);
			p_report_local_pose.getBody()->getLocalPoseRec()->setYaw(yaw);
		} catch (const std::exception& e) {
			RCLCPP_WARN(logger, "LocalPoseSensor", "Error while get yaw, pitch, roll from quaternion: %s", e.what());
		}
		// set timestamp
		ReportLocalPose::Body::LocalPoseRec::TimeStamp ts;
		// current date/time based on current system
		iop::Timestamp stamp = cmp->from_ros(cmp->now());
		ts.setDay(stamp.days);
		ts.setHour(stamp.hours);
		ts.setMinutes(stamp.minutes);
		ts.setSeconds(stamp.seconds);
		ts.setMilliseconds(stamp.milliseconds);
		p_report_local_pose.getBody()->getLocalPoseRec()->setTimeStamp(ts);
		pEvents_ReceiveFSM->get_event_handler().set_report(QueryLocalPose::ID, &p_report_local_pose);
	} catch (tf2::TransformException& ex){
		rclcpp::Clock steady_clock(RCL_STEADY_TIME);
		RCLCPP_WARN_THROTTLE(logger, steady_clock, 1000, "Could not lookup transform from '%s' to '%s': %s", p_tf_frame_robot, p_tf_frame_odom, ex.what());
	}

}

void LocalPoseSensor_ReceiveFSM::poseReceived(const geometry_msgs::msg::PoseStamped::SharedPtr pose)
{
	p_report_local_pose.getBody()->getLocalPoseRec()->setX(pose->pose.position.x);
	p_report_local_pose.getBody()->getLocalPoseRec()->setY(pose->pose.position.y);
	p_report_local_pose.getBody()->getLocalPoseRec()->setZ(pose->pose.position.z);
	try {
		tf2::Quaternion q(pose->pose.orientation.x, pose->pose.orientation.y,
				pose->pose.orientation.z, pose->pose.orientation.w);
		tf2::Matrix3x3 m(q);
		double yaw, pitch, roll;
		m.getRPY(roll, pitch, yaw);
		p_report_local_pose.getBody()->getLocalPoseRec()->setRoll(roll);
		p_report_local_pose.getBody()->getLocalPoseRec()->setPitch(pitch);
		p_report_local_pose.getBody()->getLocalPoseRec()->setYaw(yaw);
	} catch (const std::exception& e) {
		RCLCPP_WARN(logger, "LocalPoseSensor", "Error while get yaw, pitch, roll from quaternion: %s", e.what());
	}
	// set timestamp
	ReportLocalPose::Body::LocalPoseRec::TimeStamp ts;
	// current date/time based on current system
	iop::Timestamp stamp = cmp->from_ros(pose->header.stamp);
	ts.setDay(stamp.days);
	ts.setHour(stamp.hours);
	ts.setMinutes(stamp.minutes);
	ts.setSeconds(stamp.seconds);
	ts.setMilliseconds(stamp.milliseconds);
	p_report_local_pose.getBody()->getLocalPoseRec()->setTimeStamp(ts);
	pEvents_ReceiveFSM->get_event_handler().set_report(QueryLocalPose::ID, &p_report_local_pose);
}


void LocalPoseSensor_ReceiveFSM::odomReceived(const nav_msgs::msg::Odometry::SharedPtr odom)
{
	p_report_local_pose.getBody()->getLocalPoseRec()->setX(odom->pose.pose.position.x);
	p_report_local_pose.getBody()->getLocalPoseRec()->setY(odom->pose.pose.position.y);
	p_report_local_pose.getBody()->getLocalPoseRec()->setZ(odom->pose.pose.position.z);
	try {
		tf2::Quaternion q(odom->pose.pose.orientation.x, odom->pose.pose.orientation.y,
				odom->pose.pose.orientation.z, odom->pose.pose.orientation.w);
		tf2::Matrix3x3 m(q);
		double yaw, pitch, roll;
		m.getRPY(roll, pitch, yaw);
		p_report_local_pose.getBody()->getLocalPoseRec()->setRoll(roll);
		p_report_local_pose.getBody()->getLocalPoseRec()->setPitch(pitch);
		p_report_local_pose.getBody()->getLocalPoseRec()->setYaw(yaw);
	} catch (const std::exception& e) {
		RCLCPP_WARN(logger, "LocalPoseSensor", "Error while get yaw, pitch, roll from quaternion: %s", e.what());
	}
	// set timestamp
	ReportLocalPose::Body::LocalPoseRec::TimeStamp ts;
	// current date/time based on current system
	iop::Timestamp stamp(odom->header.stamp);
	ts.setDay(stamp.days);
	ts.setHour(stamp.hours);
	ts.setMinutes(stamp.minutes);
	ts.setSeconds(stamp.seconds);
	ts.setMilliseconds(stamp.milliseconds);
	p_report_local_pose.getBody()->getLocalPoseRec()->setTimeStamp(ts);
	pEvents_ReceiveFSM->get_event_handler().set_report(QueryLocalPose::ID, &p_report_local_pose);
}

}
