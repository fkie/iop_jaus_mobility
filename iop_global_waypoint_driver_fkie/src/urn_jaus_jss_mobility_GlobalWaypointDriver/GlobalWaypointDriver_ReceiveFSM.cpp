

#include "urn_jaus_jss_mobility_GlobalWaypointDriver/GlobalWaypointDriver_ReceiveFSM.h"

#include <gps_common/conversions.h>
#include <tf/transform_datatypes.h>
#include <iop_component_fkie/iop_component.h>

using namespace JTS;

namespace urn_jaus_jss_mobility_GlobalWaypointDriver
{



GlobalWaypointDriver_ReceiveFSM::GlobalWaypointDriver_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM, urn_jaus_jss_core_AccessControl::AccessControl_ReceiveFSM* pAccessControl_ReceiveFSM, urn_jaus_jss_core_Management::Management_ReceiveFSM* pManagement_ReceiveFSM)
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new GlobalWaypointDriver_ReceiveFSMContext(*this);

	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	this->pEvents_ReceiveFSM = pEvents_ReceiveFSM;
	this->pAccessControl_ReceiveFSM = pAccessControl_ReceiveFSM;
	this->pManagement_ReceiveFSM = pManagement_ReceiveFSM;
	p_travel_speed = 0.0;
	p_tv_max = 1.0;
	p_tf_frame_world = "/world";
}



GlobalWaypointDriver_ReceiveFSM::~GlobalWaypointDriver_ReceiveFSM()
{
	delete context;
}

void GlobalWaypointDriver_ReceiveFSM::setupNotifications()
{
	pManagement_ReceiveFSM->registerNotification("Receiving_Ready_NotControlled_Init", ieHandler, "InternalStateChange_To_GlobalWaypointDriver_ReceiveFSM_Receiving_Ready_NotControlled_Init", "Management_ReceiveFSM");
	pManagement_ReceiveFSM->registerNotification("Receiving_Ready_NotControlled_Standby", ieHandler, "InternalStateChange_To_GlobalWaypointDriver_ReceiveFSM_Receiving_Ready_NotControlled_Standby", "Management_ReceiveFSM");
	pManagement_ReceiveFSM->registerNotification("Receiving_Ready_NotControlled_Failure", ieHandler, "InternalStateChange_To_GlobalWaypointDriver_ReceiveFSM_Receiving_Ready_NotControlled_Failure", "Management_ReceiveFSM");
	pManagement_ReceiveFSM->registerNotification("Receiving_Ready_NotControlled_Shutdown", ieHandler, "InternalStateChange_To_GlobalWaypointDriver_ReceiveFSM_Receiving_Ready_NotControlled_Shutdown", "Management_ReceiveFSM");
	pManagement_ReceiveFSM->registerNotification("Receiving_Ready_NotControlled_Emergency", ieHandler, "InternalStateChange_To_GlobalWaypointDriver_ReceiveFSM_Receiving_Ready_NotControlled_Emergency", "Management_ReceiveFSM");
	pManagement_ReceiveFSM->registerNotification("Receiving_Ready_NotControlled", ieHandler, "InternalStateChange_To_GlobalWaypointDriver_ReceiveFSM_Receiving_Ready_NotControlled_Standby", "Management_ReceiveFSM");
	pManagement_ReceiveFSM->registerNotification("Receiving_Ready_Controlled_Standby", ieHandler, "InternalStateChange_To_GlobalWaypointDriver_ReceiveFSM_Receiving_Ready_Controlled_Standby", "Management_ReceiveFSM");
	pManagement_ReceiveFSM->registerNotification("Receiving_Ready_Controlled_Ready", ieHandler, "InternalStateChange_To_GlobalWaypointDriver_ReceiveFSM_Receiving_Ready_Controlled_Ready", "Management_ReceiveFSM");
	pManagement_ReceiveFSM->registerNotification("Receiving_Ready_Controlled_Emergency", ieHandler, "InternalStateChange_To_GlobalWaypointDriver_ReceiveFSM_Receiving_Ready_Controlled_Emergency", "Management_ReceiveFSM");
	pManagement_ReceiveFSM->registerNotification("Receiving_Ready_Controlled", ieHandler, "InternalStateChange_To_GlobalWaypointDriver_ReceiveFSM_Receiving_Ready_Controlled_Standby", "Management_ReceiveFSM");
	pManagement_ReceiveFSM->registerNotification("Receiving_Ready", ieHandler, "InternalStateChange_To_GlobalWaypointDriver_ReceiveFSM_Receiving_Ready_NotControlled_Standby", "Management_ReceiveFSM");
	pManagement_ReceiveFSM->registerNotification("Receiving", ieHandler, "InternalStateChange_To_GlobalWaypointDriver_ReceiveFSM_Receiving_Ready_NotControlled_Standby", "Management_ReceiveFSM");
	registerNotification("Receiving_Ready_NotControlled_Standby", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving_Ready_NotControlled_Standby", "GlobalWaypointDriver_ReceiveFSM");
	registerNotification("Receiving_Ready_NotControlled_Init", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving_Ready_NotControlled_Init", "GlobalWaypointDriver_ReceiveFSM");
	registerNotification("Receiving_Ready_NotControlled_Failure", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving_Ready_NotControlled_Failure", "GlobalWaypointDriver_ReceiveFSM");
	registerNotification("Receiving_Ready_NotControlled_Shutdown", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving_Ready_NotControlled_Shutdown", "GlobalWaypointDriver_ReceiveFSM");
	registerNotification("Receiving_Ready_NotControlled_Emergency", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving_Ready_NotControlled_Emergency", "GlobalWaypointDriver_ReceiveFSM");
	registerNotification("Receiving_Ready_NotControlled", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving_Ready_NotControlled", "GlobalWaypointDriver_ReceiveFSM");
	registerNotification("Receiving_Ready_Controlled_Standby", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving_Ready_Controlled_Standby", "GlobalWaypointDriver_ReceiveFSM");
	registerNotification("Receiving_Ready_Controlled_Ready", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving_Ready_Controlled_Ready", "GlobalWaypointDriver_ReceiveFSM");
	registerNotification("Receiving_Ready_Controlled_Emergency", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving_Ready_Controlled_Emergency", "GlobalWaypointDriver_ReceiveFSM");
	registerNotification("Receiving_Ready_Controlled", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving_Ready_Controlled", "GlobalWaypointDriver_ReceiveFSM");
	registerNotification("Receiving_Ready", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving_Ready", "GlobalWaypointDriver_ReceiveFSM");
	registerNotification("Receiving", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving", "GlobalWaypointDriver_ReceiveFSM");

	iop::Config cfg("~GlobalWaypointDriver");
	cfg.param("tf_frame_world", p_tf_frame_world, p_tf_frame_world);
	cfg.param("tv_max", p_tv_max, p_tv_max);

	pEvents_ReceiveFSM->get_event_handler().register_query(QueryGlobalWaypoint::ID);
	//create ROS subscriber
	p_travel_speed = 0.0;
	// p_pub_path = cfg.advertise<nav_msgs::Path>("cmd_global_waypoint", 5);
	p_pub_pose = cfg.advertise<geometry_msgs::PoseStamped>("cmd_global_pose", 5);
	p_pub_tv_max = cfg.advertise<std_msgs::Float32>("cmd_travel_speed", 5, true);
	std_msgs::Float32 ros_msg;
	ros_msg.data = p_travel_speed;
	p_pub_tv_max.publish(ros_msg);
}

void GlobalWaypointDriver_ReceiveFSM::resetTravelSpeedAction()
{
	// The travel speed is reset to zero for all transitions from the Ready State.
	p_travel_speed = 0.0;
	std_msgs::Float32 ros_msg;
	ros_msg.data = p_travel_speed;
	p_pub_tv_max.publish(ros_msg);
//	pStop();
}

void GlobalWaypointDriver_ReceiveFSM::sendReportGlobalWaypointAction(QueryGlobalWaypoint msg, Receive::Body::ReceiveRec transportData)
{
	uint16_t subsystem_id = transportData.getSourceID()->getSubsystemID();
	uint8_t node_id = transportData.getSourceID()->getNodeID();
	uint8_t component_id = transportData.getSourceID()->getComponentID();
	JausAddress sender(subsystem_id, node_id, component_id);
	ROS_DEBUG_NAMED("GlobalWaypointDriver", "sendReportGlobalWaypointAction to %d.%d.%d", subsystem_id, node_id, component_id);
	sendJausMessage(p_current_waypoint, sender);
}

void GlobalWaypointDriver_ReceiveFSM::sendReportTravelSpeedAction(QueryTravelSpeed msg, Receive::Body::ReceiveRec transportData)
{
	uint16_t subsystem_id = transportData.getSourceID()->getSubsystemID();
	uint8_t node_id = transportData.getSourceID()->getNodeID();
	uint8_t component_id = transportData.getSourceID()->getComponentID();
	JausAddress sender(subsystem_id, node_id, component_id);
	ROS_DEBUG_NAMED("GlobalWaypointDriver", "sendReportTravelSpeedAction to %d.%d.%d", subsystem_id, node_id, component_id);
	ReportTravelSpeed report;
	report.getBody()->getTravelSpeedRec()->setSpeed(p_travel_speed);
	sendJausMessage(report, sender);
}

void GlobalWaypointDriver_ReceiveFSM::setTravelSpeedAction(SetTravelSpeed msg, Receive::Body::ReceiveRec transportData)
{
	uint16_t subsystem_id = transportData.getSourceID()->getSubsystemID();
	uint8_t node_id = transportData.getSourceID()->getNodeID();
	uint8_t component_id = transportData.getSourceID()->getComponentID();
	JausAddress sender(subsystem_id, node_id, component_id);
	p_travel_speed = msg.getBody()->getTravelSpeedRec()->getSpeed();
	ROS_DEBUG_NAMED("GlobalWaypointDriver", "setTravelSpeedAction from %d.%d.%d to %.2f", subsystem_id, node_id, component_id, p_travel_speed);
	if (p_tv_max < p_travel_speed) {
		p_travel_speed = p_tv_max;
		ROS_DEBUG_NAMED("GlobalWaypointDriver", "  reduce travel speed do to parameter settings of %.2f", p_travel_speed);
	}
	std_msgs::Float32 ros_msg;
	ros_msg.data = p_travel_speed;
	p_pub_tv_max.publish(ros_msg);
	if (p_travel_speed == 0.0) {
		pStop();
	}
}

void GlobalWaypointDriver_ReceiveFSM::setWaypointAction(SetGlobalWaypoint msg, Receive::Body::ReceiveRec transportData)
{
	uint16_t subsystem_id = transportData.getSourceID()->getSubsystemID();
	uint8_t node_id = transportData.getSourceID()->getNodeID();
	uint8_t component_id = transportData.getSourceID()->getComponentID();
	JausAddress sender(subsystem_id, node_id, component_id);
	ROS_DEBUG_NAMED("GlobalWaypointDriver", "setWaypointAction from %d.%d.%d", subsystem_id, node_id, component_id);
	double lat, lon, alt = 0.0;
	double roll, pitch, yaw = 0.0;
	SetGlobalWaypoint::Body::GlobalWaypointRec *wprec = msg.getBody()->getGlobalWaypointRec();
	lat = wprec->getLatitude();
	lon = wprec->getLongitude();
	p_current_waypoint.getBody()->getGlobalWaypointRec()->setLatitude(lat);
	p_current_waypoint.getBody()->getGlobalWaypointRec()->setLongitude(lon);
	if (wprec->isAltitudeValid()) {
		alt = wprec->getAltitude();
		p_current_waypoint.getBody()->getGlobalWaypointRec()->setAltitude(alt);
	}
	if (wprec->isRollValid()) {
		roll = wprec->getRoll();
		p_current_waypoint.getBody()->getGlobalWaypointRec()->setRoll(roll);
	}
	if (wprec->isPitchValid()) {
		pitch = wprec->getPitch();
		p_current_waypoint.getBody()->getGlobalWaypointRec()->setPitch(pitch);
	}
	if (wprec->isYawValid()) {
		yaw = wprec->getYaw();
		p_current_waypoint.getBody()->getGlobalWaypointRec()->setYaw(yaw);
	}
	if (wprec->isWaypointToleranceValid()) {
		p_current_waypoint.getBody()->getGlobalWaypointRec()->setWaypointTolerance(wprec->getWaypointTolerance());
	}
	nav_msgs::Path path;
	path.header.stamp = ros::Time::now();
	path.header.frame_id = this->p_tf_frame_world;

	double northing, easting;
	std::string zone;
	gps_common::LLtoUTM(lat, lon, northing, easting, zone);
	tf::Quaternion quat;
	quat.setRPY(roll, pitch, yaw);

	if (lat > -90.0 && lon > -180.0) {
		ROS_INFO_NAMED("GlobalWaypointDriver", "new Waypoint lat: %.6f, lon: %.6f, alt: %.2f, roll: %.2f, pitch: %.2f, yaw: %.2f", lat, lon, alt, roll, pitch, yaw);
		geometry_msgs::PoseStamped pose;
		pose.header = path.header;
		pose.pose.position.x = easting;
		pose.pose.position.y = northing;
		pose.pose.position.z = alt;
		pose.pose.orientation.x = quat.x();
		pose.pose.orientation.y = quat.y();
		pose.pose.orientation.z = quat.z();
		pose.pose.orientation.w = quat.w();
		path.poses.push_back(pose);
		p_pub_pose.publish(pose);
	}

	pEvents_ReceiveFSM->get_event_handler().set_report(QueryGlobalWaypoint::ID, &p_current_waypoint);
	this->p_pub_path.publish(path);
}

void GlobalWaypointDriver_ReceiveFSM::pStop()
{
	nav_msgs::Path path;
	path.header.stamp = ros::Time::now();
	path.header.frame_id = this->p_tf_frame_world;
	ReportGlobalWaypoint waypoint;
	p_current_waypoint = waypoint;
	pEvents_ReceiveFSM->get_event_handler().set_report(QueryGlobalWaypoint::ID, &p_current_waypoint);
	this->p_pub_path.publish(path);
}


bool GlobalWaypointDriver_ReceiveFSM::isControllingClient(Receive::Body::ReceiveRec transportData)
{
	//// By default, inherited guards call the parent function.
	//// This can be replaced or modified as needed.
	return pAccessControl_ReceiveFSM->isControllingClient(transportData );
}

bool GlobalWaypointDriver_ReceiveFSM::waypointExists(QueryGlobalWaypoint msg)
{
	/// if latitude or longitude are zero the waypoint is not valid
	return (p_current_waypoint.getBody()->getGlobalWaypointRec()->getLatitude() != 0
			&& p_current_waypoint.getBody()->getGlobalWaypointRec()->getLongitude() != 0);
}

bool GlobalWaypointDriver_ReceiveFSM::waypointExists(SetTravelSpeed msg)
{
	/// if latitude or longitude are zero the waypoint is not valid
	return (p_current_waypoint.getBody()->getGlobalWaypointRec()->getLatitude() != 0
			&& p_current_waypoint.getBody()->getGlobalWaypointRec()->getLongitude() != 0);
}



};
