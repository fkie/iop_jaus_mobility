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


#include "urn_jaus_jss_mobility_VelocityStateSensor/VelocityStateSensor_ReceiveFSM.h"
#include <fkie_iop_component/iop_config.h>



using namespace JTS;

namespace urn_jaus_jss_mobility_VelocityStateSensor
{



VelocityStateSensor_ReceiveFSM::VelocityStateSensor_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM)
{

	/*
	 * If there are other variables, context must be constructed last so that all 
	 * class variables are available if an EntryAction of the InitialState of the 
	 * statemachine needs them. 
	 */
	context = new VelocityStateSensor_ReceiveFSMContext(*this);

	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	this->pEvents_ReceiveFSM = pEvents_ReceiveFSM;
}



VelocityStateSensor_ReceiveFSM::~VelocityStateSensor_ReceiveFSM() 
{
	delete context;
}

void VelocityStateSensor_ReceiveFSM::setupNotifications()
{
	pEvents_ReceiveFSM->registerNotification("Receiving_Ready", ieHandler, "InternalStateChange_To_VelocityStateSensor_ReceiveFSM_Receiving_Ready", "Events_ReceiveFSM");
	pEvents_ReceiveFSM->registerNotification("Receiving", ieHandler, "InternalStateChange_To_VelocityStateSensor_ReceiveFSM_Receiving_Ready", "Events_ReceiveFSM");
	registerNotification("Receiving_Ready", pEvents_ReceiveFSM->getHandler(), "InternalStateChange_To_Events_ReceiveFSM_Receiving_Ready", "VelocityStateSensor_ReceiveFSM");
	registerNotification("Receiving", pEvents_ReceiveFSM->getHandler(), "InternalStateChange_To_Events_ReceiveFSM_Receiving", "VelocityStateSensor_ReceiveFSM");

	iop::Config cfg("~VelocityStateSensor");
	p_odom_sub = cfg.subscribe<nav_msgs::Odometry>("odom", 1, &VelocityStateSensor_ReceiveFSM::odomReceived, this);

	pEvents_ReceiveFSM->get_event_handler().register_query(QueryVelocityState::ID);
}

void VelocityStateSensor_ReceiveFSM::SendAction(std::string arg0, Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
	ROS_DEBUG_NAMED("VelocityStateSensor", "request from %d.%d.%d",
			  transportData.getSrcSubsystemID(), transportData.getSrcNodeID(), transportData.getSrcComponentID());
	JausAddress sender = JausAddress(transportData.getSrcSubsystemID(),
									 transportData.getSrcNodeID(),
									 transportData.getSrcComponentID());

	if (strcmp(arg0.c_str(), "ReportVelocityState") == 0) {
		this->sendJausMessage(p_report_velocity_state, sender);
	}
}

void VelocityStateSensor_ReceiveFSM::odomReceived(const nav_msgs::Odometry::ConstPtr& odom)
{
	p_report_velocity_state.getBody()->getReportVelocityStateRec()->setVelocity_X(odom->twist.twist.linear.x);
	p_report_velocity_state.getBody()->getReportVelocityStateRec()->setVelocity_Y(odom->twist.twist.linear.y);
	p_report_velocity_state.getBody()->getReportVelocityStateRec()->setVelocity_Z(odom->twist.twist.linear.z);
	p_report_velocity_state.getBody()->getReportVelocityStateRec()->setRollRate(odom->twist.twist.angular.x);
	p_report_velocity_state.getBody()->getReportVelocityStateRec()->setPitchRate(odom->twist.twist.angular.y);
	p_report_velocity_state.getBody()->getReportVelocityStateRec()->setYawRate(odom->twist.twist.angular.z);
		
	// set timestamp
	ReportVelocityState::Body::ReportVelocityStateRec::TimeStamp ts;
	// current date/time based on current system
	iop::Timestamp stamp(odom->header.stamp);
	ts.setDay(stamp.days);
	ts.setHour(stamp.hours);
	ts.setMinutes(stamp.minutes);
	ts.setSeconds(stamp.seconds);
	ts.setMilliseconds(stamp.milliseconds);
	p_report_velocity_state.getBody()->getReportVelocityStateRec()->setTimeStamp(ts);
	pEvents_ReceiveFSM->get_event_handler().set_report(QueryVelocityState::ID, &p_report_velocity_state);
}




};
