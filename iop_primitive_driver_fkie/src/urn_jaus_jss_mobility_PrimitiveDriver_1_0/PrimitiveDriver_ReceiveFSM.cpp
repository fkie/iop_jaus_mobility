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


#include "urn_jaus_jss_mobility_PrimitiveDriver_1_0/PrimitiveDriver_ReceiveFSM.h"




using namespace JTS;

namespace urn_jaus_jss_mobility_PrimitiveDriver_1_0
{



PrimitiveDriver_ReceiveFSM::PrimitiveDriver_ReceiveFSM(urn_jaus_jss_core_Transport_1_0::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_Events_1_0::Events_ReceiveFSM* pEvents_ReceiveFSM, urn_jaus_jss_core_AccessControl_1_0::AccessControl_ReceiveFSM* pAccessControl_ReceiveFSM, urn_jaus_jss_core_Management_1_0::Management_ReceiveFSM* pManagement_ReceiveFSM)
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
	pManagement_ReceiveFSM->registerNotification("Receiving_Ready_NotControlled_StateA_Standby", ieHandler, "InternalStateChange_To_PrimitiveDriver_ReceiveFSM_Receiving_Ready_NotControlled_StateA_Standby", "Management_ReceiveFSM");
	pManagement_ReceiveFSM->registerNotification("Receiving_Ready_NotControlled_StateA_Init", ieHandler, "InternalStateChange_To_PrimitiveDriver_ReceiveFSM_Receiving_Ready_NotControlled_StateA_Init", "Management_ReceiveFSM");
	pManagement_ReceiveFSM->registerNotification("Receiving_Ready_NotControlled_StateA_Failure", ieHandler, "InternalStateChange_To_PrimitiveDriver_ReceiveFSM_Receiving_Ready_NotControlled_StateA_Failure", "Management_ReceiveFSM");
	pManagement_ReceiveFSM->registerNotification("Receiving_Ready_NotControlled_StateA_Shutdown", ieHandler, "InternalStateChange_To_PrimitiveDriver_ReceiveFSM_Receiving_Ready_NotControlled_StateA_Shutdown", "Management_ReceiveFSM");
	pManagement_ReceiveFSM->registerNotification("Receiving_Ready_NotControlled_StateA_Emergency", ieHandler, "InternalStateChange_To_PrimitiveDriver_ReceiveFSM_Receiving_Ready_NotControlled_StateA_Emergency", "Management_ReceiveFSM");
	pManagement_ReceiveFSM->registerNotification("Receiving_Ready_NotControlled_StateA", ieHandler, "InternalStateChange_To_PrimitiveDriver_ReceiveFSM_Receiving_Ready_NotControlled_StateA_Init", "Management_ReceiveFSM");
	pManagement_ReceiveFSM->registerNotification("Receiving_Ready_NotControlled", ieHandler, "InternalStateChange_To_PrimitiveDriver_ReceiveFSM_Receiving_Ready_NotControlled_StateA_Init", "Management_ReceiveFSM");
	pManagement_ReceiveFSM->registerNotification("Receiving_Ready_Controlled_StateB_Standby", ieHandler, "InternalStateChange_To_PrimitiveDriver_ReceiveFSM_Receiving_Ready_Controlled_StateB_Standby", "Management_ReceiveFSM");
	pManagement_ReceiveFSM->registerNotification("Receiving_Ready_Controlled_StateB_Ready", ieHandler, "InternalStateChange_To_PrimitiveDriver_ReceiveFSM_Receiving_Ready_Controlled_StateB_Ready", "Management_ReceiveFSM");
	pManagement_ReceiveFSM->registerNotification("Receiving_Ready_Controlled_StateB_Emergency", ieHandler, "InternalStateChange_To_PrimitiveDriver_ReceiveFSM_Receiving_Ready_Controlled_StateB_Emergency", "Management_ReceiveFSM");
	pManagement_ReceiveFSM->registerNotification("Receiving_Ready_Controlled_StateB", ieHandler, "InternalStateChange_To_PrimitiveDriver_ReceiveFSM_Receiving_Ready_Controlled_StateB_Standby", "Management_ReceiveFSM");
	pManagement_ReceiveFSM->registerNotification("Receiving_Ready_Controlled", ieHandler, "InternalStateChange_To_PrimitiveDriver_ReceiveFSM_Receiving_Ready_Controlled_StateB_Standby", "Management_ReceiveFSM");
	pManagement_ReceiveFSM->registerNotification("Receiving_Ready", ieHandler, "InternalStateChange_To_PrimitiveDriver_ReceiveFSM_Receiving_Ready_NotControlled_StateA_Init", "Management_ReceiveFSM");
	pManagement_ReceiveFSM->registerNotification("Receiving", ieHandler, "InternalStateChange_To_PrimitiveDriver_ReceiveFSM_Receiving_Ready_NotControlled_StateA_Init", "Management_ReceiveFSM");
	registerNotification("Receiving_Ready_NotControlled_StateA_Standby", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving_Ready_NotControlled_StateA_Standby", "PrimitiveDriver_ReceiveFSM");
	registerNotification("Receiving_Ready_NotControlled_StateA_Init", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving_Ready_NotControlled_StateA_Init", "PrimitiveDriver_ReceiveFSM");
	registerNotification("Receiving_Ready_NotControlled_StateA_Failure", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving_Ready_NotControlled_StateA_Failure", "PrimitiveDriver_ReceiveFSM");
	registerNotification("Receiving_Ready_NotControlled_StateA_Shutdown", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving_Ready_NotControlled_StateA_Shutdown", "PrimitiveDriver_ReceiveFSM");
	registerNotification("Receiving_Ready_NotControlled_StateA_Emergency", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving_Ready_NotControlled_StateA_Emergency", "PrimitiveDriver_ReceiveFSM");
	registerNotification("Receiving_Ready_NotControlled_StateA", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving_Ready_NotControlled_StateA", "PrimitiveDriver_ReceiveFSM");
	registerNotification("Receiving_Ready_NotControlled", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving_Ready_NotControlled", "PrimitiveDriver_ReceiveFSM");
	registerNotification("Receiving_Ready_Controlled_StateB_Standby", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving_Ready_Controlled_StateB_Standby", "PrimitiveDriver_ReceiveFSM");
	registerNotification("Receiving_Ready_Controlled_StateB_Ready", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving_Ready_Controlled_StateB_Ready", "PrimitiveDriver_ReceiveFSM");
	registerNotification("Receiving_Ready_Controlled_StateB_Emergency", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving_Ready_Controlled_StateB_Emergency", "PrimitiveDriver_ReceiveFSM");
	registerNotification("Receiving_Ready_Controlled_StateB", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving_Ready_Controlled_StateB", "PrimitiveDriver_ReceiveFSM");
	registerNotification("Receiving_Ready_Controlled", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving_Ready_Controlled", "PrimitiveDriver_ReceiveFSM");
	registerNotification("Receiving_Ready", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving_Ready", "PrimitiveDriver_ReceiveFSM");
	registerNotification("Receiving", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving", "PrimitiveDriver_ReceiveFSM");

//        odom_sub_ = p_nh.subscribe<nav_msgs::Odometry>("odom", 1, &PrimitiveDriver_ReceiveFSM::odomReceived, this);
	p_pnh.param("use_stamped", p_use_stamped, true);
	ROS_INFO_STREAM("[PrimitiveDriver ROS param] use_stamped: " << p_use_stamped);
	//create ROS subscriber
	if (p_use_stamped) {
	  cmd_pub_ = p_nh.advertise<geometry_msgs::TwistStamped>("cmd_vel", 5);
	} else {
	  cmd_pub_ = p_nh.advertise<geometry_msgs::Twist>("cmd_vel", 5);
	}
	p_pnh.param("max_linear_x", max_linear_x, max_linear_x);
	ROS_INFO_STREAM("[PrimitiveDriver ROS param] max_linear_x: " << max_linear_x << "m/s");
	p_pnh.param("max_linear_y", max_linear_y, max_linear_y);
	ROS_INFO_STREAM("[PrimitiveDriver ROS param] max_linear_y: " << max_linear_y << "m/s");
	p_pnh.param("max_linear_z", max_linear_z, max_linear_z);
	ROS_INFO_STREAM("[PrimitiveDriver ROS param] max_linear_z: " << max_linear_z << "m/s");

	p_pnh.param("max_angular_x", max_angular_x, max_angular_x);
	ROS_INFO_STREAM("[PrimitiveDriver ROS param] max_angular_x: " << max_angular_x << "rad");
	p_pnh.param("max_angular_y", max_angular_y, max_angular_y);
	ROS_INFO_STREAM("[PrimitiveDriver ROS param] max_angular_y: " << max_angular_y << "rad");
	p_pnh.param("max_angular_z", max_angular_z, max_angular_z);
	ROS_INFO_STREAM("[PrimitiveDriver ROS param] max_angular_z: " << max_angular_z << "rad");
}

void PrimitiveDriver_ReceiveFSM::sendReportWrenchEffortAction(QueryWrenchEffort msg, Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
  ROS_DEBUG("report WrenchEffortAction to %d.%d.%d",
            transportData.getSrcSubsystemID(), transportData.getSrcNodeID(), transportData.getSrcComponentID());
  JausAddress sender = JausAddress(transportData.getSrcSubsystemID(),
                                   transportData.getSrcNodeID(),
                                   transportData.getSrcComponentID());
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
