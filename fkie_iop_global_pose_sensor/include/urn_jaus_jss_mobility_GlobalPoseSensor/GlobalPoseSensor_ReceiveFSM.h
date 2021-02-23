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


#ifndef GLOBALPOSESENSOR_RECEIVEFSM_H
#define GLOBALPOSESENSOR_RECEIVEFSM_H

#include "JausUtils.h"
#include "InternalEvents/InternalEventHandler.h"
#include "Transport/JausTransport.h"
#include "JTSStateMachine.h"
#include "urn_jaus_jss_mobility_GlobalPoseSensor/Messages/MessageSet.h"
#include "urn_jaus_jss_mobility_GlobalPoseSensor/InternalEvents/InternalEventsSet.h"

#include "InternalEvents/Receive.h"
#include "InternalEvents/Send.h"

#include "urn_jaus_jss_core_AccessControl/AccessControl_ReceiveFSM.h"
#include "urn_jaus_jss_core_Events/Events_ReceiveFSM.h"
#include "urn_jaus_jss_core_Transport/Transport_ReceiveFSM.h"

#include "GlobalPoseSensor_ReceiveFSM_sm.h"
#include <rclcpp/rclcpp.hpp>
#include <fkie_iop_component/iop_component.hpp>

#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>


namespace urn_jaus_jss_mobility_GlobalPoseSensor
{

class DllExport GlobalPoseSensor_ReceiveFSM : public JTS::StateMachine
{
public:
	GlobalPoseSensor_ReceiveFSM(std::shared_ptr<iop::Component> cmp, urn_jaus_jss_core_AccessControl::AccessControl_ReceiveFSM* pAccessControl_ReceiveFSM, urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM, urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM);
	virtual ~GlobalPoseSensor_ReceiveFSM();

	/// Handle notifications on parent state changes
	virtual void setupNotifications();
	virtual void setupIopConfiguration();

	/// Action Methods
	virtual void SendAction(std::string arg0, Receive::Body::ReceiveRec transportData);
	virtual void updateGeomagneticPropertyAction();
	virtual void updateGlobalPoseAction();


	/// Guard Methods
	virtual bool isControllingClient(Receive::Body::ReceiveRec transportData);



	GlobalPoseSensor_ReceiveFSMContext *context;

protected:

	/// References to parent FSMs
	urn_jaus_jss_core_AccessControl::AccessControl_ReceiveFSM* pAccessControl_ReceiveFSM;
	urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM;
	urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM;

	std::shared_ptr<iop::Component> cmp;
	rclcpp::Logger logger;
	rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr p_navsatfix_sub;
	rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr p_imu_sub;
	ReportGlobalPose p_report_global_pose;
	double p_yaw, p_pitch, p_roll;
	bool p_is_ori_valid;

	void fixReceived(const sensor_msgs::msg::NavSatFix::SharedPtr fix);
	void imuReceived(const sensor_msgs::msg::Imu::SharedPtr imu);

};

}

#endif // GLOBALPOSESENSOR_RECEIVEFSM_H
