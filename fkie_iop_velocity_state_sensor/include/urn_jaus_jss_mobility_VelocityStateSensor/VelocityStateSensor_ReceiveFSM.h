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

#ifndef VELOCITYSTATESENSOR_RECEIVEFSM_H
#define VELOCITYSTATESENSOR_RECEIVEFSM_H

#include "JausUtils.h"
#include "InternalEvents/InternalEventHandler.h"
#include "Transport/JausTransport.h"
#include "JTSStateMachine.h"
#include "urn_jaus_jss_mobility_VelocityStateSensor/Messages/MessageSet.h"
#include "urn_jaus_jss_mobility_VelocityStateSensor/InternalEvents/InternalEventsSet.h"

#include "InternalEvents/Receive.h"
#include "InternalEvents/Send.h"

#include "urn_jaus_jss_core_Events/Events_ReceiveFSM.h"
#include "urn_jaus_jss_core_Transport/Transport_ReceiveFSM.h"


#include "VelocityStateSensor_ReceiveFSM_sm.h"
#include <rclcpp/rclcpp.hpp>
#include <fkie_iop_component/iop_component.hpp>

#include <nav_msgs/msg/odometry.hpp>


namespace urn_jaus_jss_mobility_VelocityStateSensor
{
	
class DllExport VelocityStateSensor_ReceiveFSM : public JTS::StateMachine
{
public:
	VelocityStateSensor_ReceiveFSM(std::shared_ptr<iop::Component> cmp, urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM, urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM);
	virtual ~VelocityStateSensor_ReceiveFSM();

	/// Handle notifications on parent state changes
	virtual void setupNotifications();
	virtual void setupIopConfiguration();

	/// Action Methods
	virtual void SendAction(std::string arg0, Receive::Body::ReceiveRec transportData);


	/// Guard Methods

	
	
	VelocityStateSensor_ReceiveFSMContext *context;
	
protected:

	/// References to parent FSMs
	urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM;
	urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM;

	std::shared_ptr<iop::Component> cmp;
	rclcpp::Logger logger;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr p_odom_sub;
	ReportVelocityState p_report_velocity_state;

	void odomReceived(const nav_msgs::msg::Odometry::SharedPtr odom);
};

}

#endif // VELOCITYSTATESENSOR_RECEIVEFSM_H
