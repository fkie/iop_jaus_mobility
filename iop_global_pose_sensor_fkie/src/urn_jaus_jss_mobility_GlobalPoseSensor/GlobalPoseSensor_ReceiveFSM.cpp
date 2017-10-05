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

#include "urn_jaus_jss_mobility_GlobalPoseSensor/GlobalPoseSensor_ReceiveFSM.h"
#include <cmath>
#include <tf/transform_datatypes.h>
#include <ctime>
#include <iop_builder_fkie/timestamp.h>
#include <iop_component_fkie/iop_config.h>


using namespace JTS;

namespace urn_jaus_jss_mobility_GlobalPoseSensor
{



GlobalPoseSensor_ReceiveFSM::GlobalPoseSensor_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM, urn_jaus_jss_core_AccessControl::AccessControl_ReceiveFSM* pAccessControl_ReceiveFSM)
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new GlobalPoseSensor_ReceiveFSMContext(*this);

	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	this->pEvents_ReceiveFSM = pEvents_ReceiveFSM;
	this->pAccessControl_ReceiveFSM = pAccessControl_ReceiveFSM;
	this->p_is_ori_valid = false;
	p_yaw = NAN;
	p_pitch = NAN;
	p_roll = NAN;
}



GlobalPoseSensor_ReceiveFSM::~GlobalPoseSensor_ReceiveFSM()
{
	delete context;
}

void GlobalPoseSensor_ReceiveFSM::setupNotifications()
{
	pAccessControl_ReceiveFSM->registerNotification("Receiving_Ready_NotControlled", ieHandler, "InternalStateChange_To_GlobalPoseSensor_ReceiveFSM_Receiving_Ready_NotControlled", "AccessControl_ReceiveFSM");
	pAccessControl_ReceiveFSM->registerNotification("Receiving_Ready_Controlled", ieHandler, "InternalStateChange_To_GlobalPoseSensor_ReceiveFSM_Receiving_Ready_Controlled", "AccessControl_ReceiveFSM");
	pAccessControl_ReceiveFSM->registerNotification("Receiving_Ready", ieHandler, "InternalStateChange_To_GlobalPoseSensor_ReceiveFSM_Receiving_Ready_NotControlled", "AccessControl_ReceiveFSM");
	pAccessControl_ReceiveFSM->registerNotification("Receiving", ieHandler, "InternalStateChange_To_GlobalPoseSensor_ReceiveFSM_Receiving_Ready_NotControlled", "AccessControl_ReceiveFSM");
	registerNotification("Receiving_Ready_NotControlled", pAccessControl_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControl_ReceiveFSM_Receiving_Ready_NotControlled", "GlobalPoseSensor_ReceiveFSM");
	registerNotification("Receiving_Ready_Controlled", pAccessControl_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControl_ReceiveFSM_Receiving_Ready_Controlled", "GlobalPoseSensor_ReceiveFSM");
	registerNotification("Receiving_Ready", pAccessControl_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControl_ReceiveFSM_Receiving_Ready", "GlobalPoseSensor_ReceiveFSM");
	registerNotification("Receiving", pAccessControl_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControl_ReceiveFSM_Receiving", "GlobalPoseSensor_ReceiveFSM");
	iop::Config cfg("~GlobalPoseSensor");
	p_navsatfix_sub = cfg.subscribe<sensor_msgs::NavSatFix>("fix", 1, &GlobalPoseSensor_ReceiveFSM::fixReceived, this);
	p_imu_sub = cfg.subscribe<sensor_msgs::Imu>("imu", 1, &GlobalPoseSensor_ReceiveFSM::imuReceived, this);
	pEvents_ReceiveFSM->get_event_handler().register_query(QueryGlobalPose::ID);
}

void GlobalPoseSensor_ReceiveFSM::SendAction(std::string arg0, Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
	ROS_DEBUG("request %s from %d.%d.%d", arg0.c_str(),
			  transportData.getSrcSubsystemID(), transportData.getSrcNodeID(), transportData.getSrcComponentID());
	JausAddress sender = JausAddress(transportData.getSrcSubsystemID(),
									 transportData.getSrcNodeID(),
									 transportData.getSrcComponentID());
	if (strcmp(arg0.c_str(), "ReportGlobalPose") == 0) {
	  this->sendJausMessage(p_report_global_pose, sender);
	} else if (strcmp(arg0.c_str(), "ReportGeomagneticProperty") == 0) {
	}
}

void GlobalPoseSensor_ReceiveFSM::updateGeomagneticPropertyAction()
{
	/// Insert User Code HERE
}

void GlobalPoseSensor_ReceiveFSM::updateGlobalPoseAction()
{
	/// Insert User Code HERE
}



bool GlobalPoseSensor_ReceiveFSM::isControllingClient(Receive::Body::ReceiveRec transportData)
{
	//// By default, inherited guards call the parent function.
	//// This can be replaced or modified as needed.
	return pAccessControl_ReceiveFSM->isControllingClient(transportData );
}

void GlobalPoseSensor_ReceiveFSM::fixReceived(const sensor_msgs::NavSatFix::ConstPtr& fix)
{
	if (fix->status.status != -1) {
		p_report_global_pose.getBody()->getGlobalPoseRec()->setLatitude(fix->latitude);
		p_report_global_pose.getBody()->getGlobalPoseRec()->setLongitude(fix->longitude);
		p_report_global_pose.getBody()->getGlobalPoseRec()->setAltitude(fix->altitude);
		if (p_is_ori_valid) {
			p_report_global_pose.getBody()->getGlobalPoseRec()->setYaw(p_yaw);
			p_report_global_pose.getBody()->getGlobalPoseRec()->setPitch(p_pitch);
			p_report_global_pose.getBody()->getGlobalPoseRec()->setRoll(p_roll);
		}
		// set timestamp
		ReportGlobalPose::Body::GlobalPoseRec::TimeStamp ts;
		// current date/time based on current system
		iop::Timestamp stamp(fix->header.stamp);
		ts.setDay(stamp.days);
		ts.setHour(stamp.hours);
		ts.setMinutes(stamp.minutes);
		ts.setSeconds(stamp.seconds);
		ts.setMilliseconds(stamp.milliseconds);
		p_report_global_pose.getBody()->getGlobalPoseRec()->setTimeStamp(ts);
		pEvents_ReceiveFSM->get_event_handler().set_report(QueryGlobalPose::ID, &p_report_global_pose);
	}
}

void GlobalPoseSensor_ReceiveFSM::imuReceived(const sensor_msgs::Imu::ConstPtr& imu)
{
	try {
		tf::Quaternion q(imu->orientation.x, imu->orientation.y, imu->orientation.z, imu->orientation.w);
		tf::Matrix3x3 m(q);
		m.getRPY(p_roll, p_pitch, p_yaw);
		p_is_ori_valid = true;
	} catch (const std::exception& e) {
		ROS_WARN("Error while get yaw, pitch, roll from quaternion: %s", e.what());
	}
}

};
