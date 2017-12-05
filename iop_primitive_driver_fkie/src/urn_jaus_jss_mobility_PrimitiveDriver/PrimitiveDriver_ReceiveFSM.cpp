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


#include "urn_jaus_jss_mobility_PrimitiveDriver/PrimitiveDriver_ReceiveFSM.h"
#include <iop_component_fkie/iop_config.h>



using namespace JTS;

namespace urn_jaus_jss_mobility_PrimitiveDriver
{



PrimitiveDriver_ReceiveFSM::PrimitiveDriver_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM, urn_jaus_jss_core_AccessControl::AccessControl_ReceiveFSM* pAccessControl_ReceiveFSM, urn_jaus_jss_core_Management::Management_ReceiveFSM* pManagement_ReceiveFSM)
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new PrimitiveDriver_ReceiveFSMContext(*this);

	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	this->pEvents_ReceiveFSM = pEvents_ReceiveFSM;
	this->pAccessControl_ReceiveFSM = pAccessControl_ReceiveFSM;
	this->pManagement_ReceiveFSM = pManagement_ReceiveFSM;
	max_linear_x = 3.0;
	max_linear_y = 0;
	max_linear_z = 0;
	max_angular_x = 0.0;
	max_angular_y = 0.0;
	max_angular_z = 1.5;
	p_use_stamped = true;
	p_pnh = ros::NodeHandle("~");
}



PrimitiveDriver_ReceiveFSM::~PrimitiveDriver_ReceiveFSM()
{
	delete context;
}

void PrimitiveDriver_ReceiveFSM::setupNotifications()
{
	pManagement_ReceiveFSM->registerNotification("Receiving_Ready_NotControlled_Init", ieHandler, "InternalStateChange_To_PrimitiveDriver_ReceiveFSM_Receiving_Ready_NotControlled_Init", "Management_ReceiveFSM");
	pManagement_ReceiveFSM->registerNotification("Receiving_Ready_NotControlled_Standby", ieHandler, "InternalStateChange_To_PrimitiveDriver_ReceiveFSM_Receiving_Ready_NotControlled_Standby", "Management_ReceiveFSM");
	pManagement_ReceiveFSM->registerNotification("Receiving_Ready_NotControlled_Failure", ieHandler, "InternalStateChange_To_PrimitiveDriver_ReceiveFSM_Receiving_Ready_NotControlled_Failure", "Management_ReceiveFSM");
	pManagement_ReceiveFSM->registerNotification("Receiving_Ready_NotControlled_Shutdown", ieHandler, "InternalStateChange_To_PrimitiveDriver_ReceiveFSM_Receiving_Ready_NotControlled_Shutdown", "Management_ReceiveFSM");
	pManagement_ReceiveFSM->registerNotification("Receiving_Ready_NotControlled_Emergency", ieHandler, "InternalStateChange_To_PrimitiveDriver_ReceiveFSM_Receiving_Ready_NotControlled_Emergency", "Management_ReceiveFSM");
	pManagement_ReceiveFSM->registerNotification("Receiving_Ready_NotControlled", ieHandler, "InternalStateChange_To_PrimitiveDriver_ReceiveFSM_Receiving_Ready_NotControlled_Standby", "Management_ReceiveFSM");
	pManagement_ReceiveFSM->registerNotification("Receiving_Ready_Controlled_Standby", ieHandler, "InternalStateChange_To_PrimitiveDriver_ReceiveFSM_Receiving_Ready_Controlled_Standby", "Management_ReceiveFSM");
	pManagement_ReceiveFSM->registerNotification("Receiving_Ready_Controlled_Ready", ieHandler, "InternalStateChange_To_PrimitiveDriver_ReceiveFSM_Receiving_Ready_Controlled_Ready", "Management_ReceiveFSM");
	pManagement_ReceiveFSM->registerNotification("Receiving_Ready_Controlled_Emergency", ieHandler, "InternalStateChange_To_PrimitiveDriver_ReceiveFSM_Receiving_Ready_Controlled_Emergency", "Management_ReceiveFSM");
	pManagement_ReceiveFSM->registerNotification("Receiving_Ready_Controlled", ieHandler, "InternalStateChange_To_PrimitiveDriver_ReceiveFSM_Receiving_Ready_Controlled_Standby", "Management_ReceiveFSM");
	pManagement_ReceiveFSM->registerNotification("Receiving_Ready", ieHandler, "InternalStateChange_To_PrimitiveDriver_ReceiveFSM_Receiving_Ready_NotControlled_Standby", "Management_ReceiveFSM");
	pManagement_ReceiveFSM->registerNotification("Receiving", ieHandler, "InternalStateChange_To_PrimitiveDriver_ReceiveFSM_Receiving_Ready_NotControlled_Standby", "Management_ReceiveFSM");
	registerNotification("Receiving_Ready_NotControlled_Standby", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving_Ready_NotControlled_Standby", "PrimitiveDriver_ReceiveFSM");
	registerNotification("Receiving_Ready_NotControlled_Init", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving_Ready_NotControlled_Init", "PrimitiveDriver_ReceiveFSM");
	registerNotification("Receiving_Ready_NotControlled_Failure", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving_Ready_NotControlled_Failure", "PrimitiveDriver_ReceiveFSM");
	registerNotification("Receiving_Ready_NotControlled_Shutdown", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving_Ready_NotControlled_Shutdown", "PrimitiveDriver_ReceiveFSM");
	registerNotification("Receiving_Ready_NotControlled_Emergency", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving_Ready_NotControlled_Emergency", "PrimitiveDriver_ReceiveFSM");
	registerNotification("Receiving_Ready_NotControlled", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving_Ready_NotControlled", "PrimitiveDriver_ReceiveFSM");
	registerNotification("Receiving_Ready_Controlled_Standby", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving_Ready_Controlled_Standby", "PrimitiveDriver_ReceiveFSM");
	registerNotification("Receiving_Ready_Controlled_Ready", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving_Ready_Controlled_Ready", "PrimitiveDriver_ReceiveFSM");
	registerNotification("Receiving_Ready_Controlled_Emergency", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving_Ready_Controlled_Emergency", "PrimitiveDriver_ReceiveFSM");
	registerNotification("Receiving_Ready_Controlled", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving_Ready_Controlled", "PrimitiveDriver_ReceiveFSM");
	registerNotification("Receiving_Ready", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving_Ready", "PrimitiveDriver_ReceiveFSM");
	registerNotification("Receiving", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving", "PrimitiveDriver_ReceiveFSM");

	iop::Config cfg("~PrimitiveDriver");
	cfg.param("max_linear_x", max_linear_x, max_linear_x, true, false, "m/s");
	cfg.param("max_linear_y", max_linear_y, max_linear_y, true, false, "m/s");
	cfg.param("max_linear_z", max_linear_z, max_linear_z, true, false, "m/s");
	cfg.param("max_angular_x", max_angular_x, max_angular_x, true, false, "rad");
	cfg.param("max_angular_y", max_angular_y, max_angular_y, true, false, "rad");
	cfg.param("max_angular_z", max_angular_z, max_angular_z, true, false, "rad");
	cfg.param("use_stamped", p_use_stamped, p_use_stamped);
//        odom_sub_ = p_nh.subscribe<nav_msgs::Odometry>("odom", 1, &PrimitiveDriver_ReceiveFSM::odomReceived, this);
	//create ROS subscriber
	if (p_use_stamped) {
		cmd_pub_ = cfg.advertise<geometry_msgs::TwistStamped>("cmd_vel", 5);
	} else {
		cmd_pub_ = cfg.advertise<geometry_msgs::Twist>("cmd_vel", 5);
	}
}

void PrimitiveDriver_ReceiveFSM::sendReportWrenchEffortAction(QueryWrenchEffort msg, Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
	JausAddress sender = transportData.getAddress();
	ROS_DEBUG_NAMED("PrimitiveDriver", "report WrenchEffortAction to %s", sender.str().c_str());
	ReportWrenchEffort response;
	response.getBody()->setWrenchEffortRec(current_wrench_effort_);
	this->sendJausMessage(response, sender);
}

void PrimitiveDriver_ReceiveFSM::setWrenchEffortAction(SetWrenchEffort msg, Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
	SetWrenchEffort::Body::WrenchEffortRec *we= msg.getBody()->getWrenchEffortRec();
	ReportWrenchEffort::Body::WrenchEffortRec new_wrench_effort;
	geometry_msgs::Twist cmd_vel;
	if (we->isPropulsiveLinearEffortXValid()) {
		cmd_vel.linear.x = we->getPropulsiveLinearEffortX() / 100.0 * max_linear_x ;
		new_wrench_effort.setPropulsiveLinearEffortX(we->getPropulsiveLinearEffortX());
	}
	if (we->isPropulsiveLinearEffortYValid()) {
		cmd_vel.linear.y = we->getPropulsiveLinearEffortY() / 100.0 * max_linear_y;
		new_wrench_effort.setPropulsiveLinearEffortY(we->getPropulsiveLinearEffortY());
	}
	if (we->isPropulsiveLinearEffortZValid()) {
		cmd_vel.linear.z = we->getPropulsiveLinearEffortZ() / 100.0 * max_linear_z;
		new_wrench_effort.setPropulsiveLinearEffortZ(we->getPropulsiveLinearEffortZ());
	}
	if (we->isPropulsiveRotationalEffortXValid()) {
		cmd_vel.angular.x = we->getPropulsiveRotationalEffortX() / 100.0 * max_angular_x;
		new_wrench_effort.setPropulsiveRotationalEffortX(we->getPropulsiveRotationalEffortX());
	}
	if (we->isPropulsiveRotationalEffortYValid()) {
		cmd_vel.angular.y = we->getPropulsiveRotationalEffortY() / 100.0 * max_angular_y;
		new_wrench_effort.setPropulsiveRotationalEffortY(we->getPropulsiveRotationalEffortY());
	}
	if (we->isPropulsiveRotationalEffortZValid()) {
		cmd_vel.angular.z = we->getPropulsiveRotationalEffortZ() / 100.0 * max_angular_z;
		new_wrench_effort.setPropulsiveRotationalEffortZ(we->getPropulsiveRotationalEffortZ());
	}
	current_wrench_effort_ = new_wrench_effort;
	if (p_use_stamped) {
		// since the jaus message does not have time stamp, we use current time
		geometry_msgs::TwistStamped cmd_vel_stamped;
		cmd_vel_stamped.header.stamp = ros::Time::now();
		cmd_vel_stamped.twist = cmd_vel;
		cmd_pub_.publish(cmd_vel_stamped);
	} else {
		cmd_pub_.publish(cmd_vel);
	}
	pAccessControl_ReceiveFSM->resetTimerAction();
}

void PrimitiveDriver_ReceiveFSM::stopMotionAction()
{
	/// Insert User Code HERE
	geometry_msgs::Twist cmd_vel;
	cmd_vel.linear.x = 0;
	cmd_vel.linear.y = 0;
	cmd_vel.linear.z = 0;
	cmd_vel.angular.x = 0;
	cmd_vel.angular.y = 0;
	cmd_vel.angular.z = 0;
	if (p_use_stamped) {
		// since the jaus message does not have time stamp, we use current time
		geometry_msgs::TwistStamped cmd_vel_stamped;
		cmd_vel_stamped.header.stamp = ros::Time::now();
		cmd_vel_stamped.twist = cmd_vel;
		cmd_pub_.publish(cmd_vel_stamped);
	} else {
		cmd_pub_.publish(cmd_vel);
	}
}

bool PrimitiveDriver_ReceiveFSM::isControllingClient(Receive::Body::ReceiveRec transportData)
{
	//// By default, inherited guards call the parent function.
	//// This can be replaced or modified as needed.
	return pAccessControl_ReceiveFSM->isControllingClient(transportData );
}

void PrimitiveDriver_ReceiveFSM::odomReceived(const nav_msgs::Odometry::ConstPtr& odom)
{
	//TODO: store odometry for odometry service
}

};
