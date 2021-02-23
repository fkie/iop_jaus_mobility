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


#ifndef PRIMITIVEDRIVER_RECEIVEFSM_H
#define PRIMITIVEDRIVER_RECEIVEFSM_H

#include "JausUtils.h"
#include "InternalEvents/InternalEventHandler.h"
#include "Transport/JausTransport.h"
#include "JTSStateMachine.h"
#include "urn_jaus_jss_mobility_PrimitiveDriver/Messages/MessageSet.h"
#include "urn_jaus_jss_mobility_PrimitiveDriver/InternalEvents/InternalEventsSet.h"

#include "InternalEvents/Receive.h"
#include "InternalEvents/Send.h"

#include "urn_jaus_jss_core_Management/Management_ReceiveFSM.h"
#include "urn_jaus_jss_core_AccessControl/AccessControl_ReceiveFSM.h"
#include "urn_jaus_jss_core_Events/Events_ReceiveFSM.h"
#include "urn_jaus_jss_core_Transport/Transport_ReceiveFSM.h"


#include "PrimitiveDriver_ReceiveFSM_sm.h"
#include <rclcpp/rclcpp.hpp>
#include <fkie_iop_component/iop_component.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>


#include "PrimitiveDriver_ReceiveFSM_sm.h"

namespace urn_jaus_jss_mobility_PrimitiveDriver
{

class DllExport PrimitiveDriver_ReceiveFSM : public JTS::StateMachine
{
public:
	PrimitiveDriver_ReceiveFSM(std::shared_ptr<iop::Component> cmp, urn_jaus_jss_core_Management::Management_ReceiveFSM* pManagement_ReceiveFSM, urn_jaus_jss_core_AccessControl::AccessControl_ReceiveFSM* pAccessControl_ReceiveFSM, urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM, urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM);
	virtual ~PrimitiveDriver_ReceiveFSM();

	/// Handle notifications on parent state changes
	virtual void setupNotifications();
	virtual void setupIopConfiguration();

	/// Action Methods
	virtual void sendReportWrenchEffortAction(QueryWrenchEffort msg, Receive::Body::ReceiveRec transportData);
	virtual void setWrenchEffortAction(SetWrenchEffort msg, Receive::Body::ReceiveRec transportData);
	virtual void stopMotionAction();


	/// Guard Methods
	virtual bool isControllingClient(Receive::Body::ReceiveRec transportData);



	PrimitiveDriver_ReceiveFSMContext *context;

protected:

	/// References to parent FSMs
	urn_jaus_jss_core_Management::Management_ReceiveFSM* pManagement_ReceiveFSM;
	urn_jaus_jss_core_AccessControl::AccessControl_ReceiveFSM* pAccessControl_ReceiveFSM;
	urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM;
	urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM;

	std::shared_ptr<iop::Component> cmp;
	rclcpp::Logger logger;
//	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
	rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_stamped_pub_;
	ReportWrenchEffort::Body::WrenchEffortRec current_wrench_effort_;
	double max_linear_x;
	double max_linear_y;
	double max_linear_z;
	double max_angular_x;
	double max_angular_y;
	double max_angular_z;

	void odomReceived(const nav_msgs::msg::Odometry::SharedPtr odom);
};

}

#endif // PRIMITIVEDRIVER_RECEIVEFSM_H
