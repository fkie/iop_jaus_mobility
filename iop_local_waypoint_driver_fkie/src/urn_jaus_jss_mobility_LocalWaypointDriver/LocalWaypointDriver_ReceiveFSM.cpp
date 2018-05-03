

#include "urn_jaus_jss_mobility_LocalWaypointDriver/LocalWaypointDriver_ReceiveFSM.h"

#include <tf/transform_datatypes.h>
#include <iop_component_fkie/iop_component.h>

using namespace JTS;

namespace urn_jaus_jss_mobility_LocalWaypointDriver
{



LocalWaypointDriver_ReceiveFSM::LocalWaypointDriver_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM, urn_jaus_jss_core_AccessControl::AccessControl_ReceiveFSM* pAccessControl_ReceiveFSM, urn_jaus_jss_core_Management::Management_ReceiveFSM* pManagement_ReceiveFSM)
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new LocalWaypointDriver_ReceiveFSMContext(*this);

	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	this->pEvents_ReceiveFSM = pEvents_ReceiveFSM;
	this->pAccessControl_ReceiveFSM = pAccessControl_ReceiveFSM;
	this->pManagement_ReceiveFSM = pManagement_ReceiveFSM;
	p_travel_speed = 0.0;
	p_tv_max = 1.0;
	p_tf_frame_robot = "base_link";
	p_current_waypoint.getBody()->getLocalWaypointRec()->setX(0.0);
	p_current_waypoint.getBody()->getLocalWaypointRec()->setY(0.0);
	p_current_waypoint.getBody()->getLocalWaypointRec()->setY(0.0);
}



LocalWaypointDriver_ReceiveFSM::~LocalWaypointDriver_ReceiveFSM()
{
	delete context;
}

void LocalWaypointDriver_ReceiveFSM::setupNotifications()
{
	pManagement_ReceiveFSM->registerNotification("Receiving_Ready_NotControlled_Init", ieHandler, "InternalStateChange_To_LocalWaypointDriver_ReceiveFSM_Receiving_Ready_NotControlled_Init", "Management_ReceiveFSM");
	pManagement_ReceiveFSM->registerNotification("Receiving_Ready_NotControlled_Standby", ieHandler, "InternalStateChange_To_LocalWaypointDriver_ReceiveFSM_Receiving_Ready_NotControlled_Standby", "Management_ReceiveFSM");
	pManagement_ReceiveFSM->registerNotification("Receiving_Ready_NotControlled_Failure", ieHandler, "InternalStateChange_To_LocalWaypointDriver_ReceiveFSM_Receiving_Ready_NotControlled_Failure", "Management_ReceiveFSM");
	pManagement_ReceiveFSM->registerNotification("Receiving_Ready_NotControlled_Shutdown", ieHandler, "InternalStateChange_To_LocalWaypointDriver_ReceiveFSM_Receiving_Ready_NotControlled_Shutdown", "Management_ReceiveFSM");
	pManagement_ReceiveFSM->registerNotification("Receiving_Ready_NotControlled_Emergency", ieHandler, "InternalStateChange_To_LocalWaypointDriver_ReceiveFSM_Receiving_Ready_NotControlled_Emergency", "Management_ReceiveFSM");
	pManagement_ReceiveFSM->registerNotification("Receiving_Ready_NotControlled", ieHandler, "InternalStateChange_To_LocalWaypointDriver_ReceiveFSM_Receiving_Ready_NotControlled_Standby", "Management_ReceiveFSM");
	pManagement_ReceiveFSM->registerNotification("Receiving_Ready_Controlled_Standby", ieHandler, "InternalStateChange_To_LocalWaypointDriver_ReceiveFSM_Receiving_Ready_Controlled_Standby", "Management_ReceiveFSM");
	pManagement_ReceiveFSM->registerNotification("Receiving_Ready_Controlled_Ready", ieHandler, "InternalStateChange_To_LocalWaypointDriver_ReceiveFSM_Receiving_Ready_Controlled_Ready", "Management_ReceiveFSM");
	pManagement_ReceiveFSM->registerNotification("Receiving_Ready_Controlled_Emergency", ieHandler, "InternalStateChange_To_LocalWaypointDriver_ReceiveFSM_Receiving_Ready_Controlled_Emergency", "Management_ReceiveFSM");
	pManagement_ReceiveFSM->registerNotification("Receiving_Ready_Controlled", ieHandler, "InternalStateChange_To_LocalWaypointDriver_ReceiveFSM_Receiving_Ready_Controlled_Standby", "Management_ReceiveFSM");
	pManagement_ReceiveFSM->registerNotification("Receiving_Ready", ieHandler, "InternalStateChange_To_LocalWaypointDriver_ReceiveFSM_Receiving_Ready_NotControlled_Standby", "Management_ReceiveFSM");
	pManagement_ReceiveFSM->registerNotification("Receiving", ieHandler, "InternalStateChange_To_LocalWaypointDriver_ReceiveFSM_Receiving_Ready_NotControlled_Standby", "Management_ReceiveFSM");
	registerNotification("Receiving_Ready_NotControlled_Standby", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving_Ready_NotControlled_Standby", "LocalWaypointDriver_ReceiveFSM");
	registerNotification("Receiving_Ready_NotControlled_Init", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving_Ready_NotControlled_Init", "LocalWaypointDriver_ReceiveFSM");
	registerNotification("Receiving_Ready_NotControlled_Failure", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving_Ready_NotControlled_Failure", "LocalWaypointDriver_ReceiveFSM");
	registerNotification("Receiving_Ready_NotControlled_Shutdown", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving_Ready_NotControlled_Shutdown", "LocalWaypointDriver_ReceiveFSM");
	registerNotification("Receiving_Ready_NotControlled_Emergency", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving_Ready_NotControlled_Emergency", "LocalWaypointDriver_ReceiveFSM");
	registerNotification("Receiving_Ready_NotControlled", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving_Ready_NotControlled", "LocalWaypointDriver_ReceiveFSM");
	registerNotification("Receiving_Ready_Controlled_Standby", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving_Ready_Controlled_Standby", "LocalWaypointDriver_ReceiveFSM");
	registerNotification("Receiving_Ready_Controlled_Ready", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving_Ready_Controlled_Ready", "LocalWaypointDriver_ReceiveFSM");
	registerNotification("Receiving_Ready_Controlled_Emergency", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving_Ready_Controlled_Emergency", "LocalWaypointDriver_ReceiveFSM");
	registerNotification("Receiving_Ready_Controlled", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving_Ready_Controlled", "LocalWaypointDriver_ReceiveFSM");
	registerNotification("Receiving_Ready", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving_Ready", "LocalWaypointDriver_ReceiveFSM");
	registerNotification("Receiving", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving", "LocalWaypointDriver_ReceiveFSM");

	iop::Config cfg("~LocalWaypointDriver");
	cfg.param("tf_frame_robot", p_tf_frame_robot, p_tf_frame_robot);
	cfg.param("tv_max", p_tv_max, p_tv_max);

	pEvents_ReceiveFSM->get_event_handler().register_query(QueryLocalWaypoint::ID);
	//create ROS subscriber
	p_travel_speed = 0.0;
	p_pub_pose = cfg.advertise<geometry_msgs::PoseStamped>("cmd_local_pose", 5);
	p_pub_tv_max = cfg.advertise<std_msgs::Float32>("cmd_travel_speed", 5, true);
	p_sub_finished = cfg.subscribe<std_msgs::Bool>("local_way_point_reached", 5, &LocalWaypointDriver_ReceiveFSM::pRosFinished, this);
	std_msgs::Float32 ros_msg;
	ros_msg.data = p_travel_speed;
	p_pub_tv_max.publish(ros_msg);
}

void LocalWaypointDriver_ReceiveFSM::resetTravelSpeedAction()
{
	// The travel speed is reset to zero for all transitions from the Ready State.
	p_travel_speed = 0.0;
	std_msgs::Float32 ros_msg;
	ros_msg.data = p_travel_speed;
	p_pub_tv_max.publish(ros_msg);
//	pStop();
}

void LocalWaypointDriver_ReceiveFSM::sendReportLocalWaypointAction(QueryLocalWaypoint msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress sender = transportData.getAddress();
	ROS_DEBUG_NAMED("LocalWaypointDriver", "sendReportLocalWaypointAction to %s", sender.str().c_str());
	sendJausMessage(p_current_waypoint, sender);
}

void LocalWaypointDriver_ReceiveFSM::sendReportTravelSpeedAction(QueryTravelSpeed msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress sender = transportData.getAddress();
	ROS_DEBUG_NAMED("LocalWaypointDriver", "sendReportTravelSpeedAction to %s", sender.str().c_str());
	ReportTravelSpeed report;
	report.getBody()->getTravelSpeedRec()->setSpeed(p_travel_speed);
	sendJausMessage(report, sender);
}

void LocalWaypointDriver_ReceiveFSM::setTravelSpeedAction(SetTravelSpeed msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress sender = transportData.getAddress();
	p_travel_speed = msg.getBody()->getTravelSpeedRec()->getSpeed();
	ROS_DEBUG_NAMED("LocalWaypointDriver", "setTravelSpeedAction from %s to %.2f", sender.str().c_str(), p_travel_speed);
	if (p_tv_max < p_travel_speed) {
		p_travel_speed = p_tv_max;
		ROS_DEBUG_NAMED("LocalWaypointDriver", "  reduce travel speed do to parameter settings of %.2f", p_travel_speed);
	}
	std_msgs::Float32 ros_msg;
	ros_msg.data = p_travel_speed;
	p_pub_tv_max.publish(ros_msg);
	if (p_travel_speed == 0.0) {
		pStop();
	}
}

void LocalWaypointDriver_ReceiveFSM::setWaypointAction(SetLocalWaypoint msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress sender = transportData.getAddress();
	ROS_DEBUG_NAMED("LocalWaypointDriver", "setWaypointAction from %s", sender.str().c_str());
	double roll, pitch, yaw = 0.0;
	SetLocalWaypoint::Body::LocalWaypointRec *wprec = msg.getBody()->getLocalWaypointRec();
	double x = wprec->getX();
	double y = wprec->getY();
	double z = 0;
	p_current_waypoint.getBody()->getLocalWaypointRec()->setX(x);
	p_current_waypoint.getBody()->getLocalWaypointRec()->setY(y);
	if (wprec->isZValid()) {
		z = wprec->getZ();
		p_current_waypoint.getBody()->getLocalWaypointRec()->setZ(z);
	}
	if (wprec->isRollValid()) {
		roll = wprec->getRoll();
		p_current_waypoint.getBody()->getLocalWaypointRec()->setRoll(roll);
	}
	if (wprec->isPitchValid()) {
		pitch = wprec->getPitch();
		p_current_waypoint.getBody()->getLocalWaypointRec()->setPitch(pitch);
	}
	if (wprec->isYawValid()) {
		yaw = wprec->getYaw();
		p_current_waypoint.getBody()->getLocalWaypointRec()->setYaw(yaw);
	}
	if (wprec->isWaypointToleranceValid()) {
		p_current_waypoint.getBody()->getLocalWaypointRec()->setWaypointTolerance(wprec->getWaypointTolerance());
	}
	nav_msgs::Path path;
	path.header.stamp = ros::Time::now();
	path.header.frame_id = this->p_tf_frame_robot;

	tf::Quaternion quat = tf::createQuaternionFromRPY(roll, pitch, yaw);

	ROS_INFO_NAMED("LocalWaypointDriver", "new Waypoint x: %.6f, y: %.6f, z: %.2f, roll: %.2f, pitch: %.2f, yaw: %.2f", x, y, z, roll, pitch, yaw);
	geometry_msgs::PoseStamped pose;
	pose.header = path.header;
	pose.pose.position.x = x;
	pose.pose.position.y = y;
	pose.pose.position.z = z;
	pose.pose.orientation.x = quat.x();
	pose.pose.orientation.y = quat.y();
	pose.pose.orientation.z = quat.z();
	pose.pose.orientation.w = quat.w();
	path.poses.push_back(pose);
	p_pub_pose.publish(pose);
	pEvents_ReceiveFSM->get_event_handler().set_report(QueryLocalWaypoint::ID, &p_current_waypoint);
}

void LocalWaypointDriver_ReceiveFSM::pStop()
{
	nav_msgs::Path path;
	path.header.stamp = ros::Time::now();
	path.header.frame_id = this->p_tf_frame_robot;
	ReportLocalWaypoint waypoint;
	p_current_waypoint = waypoint;
	p_current_waypoint.getBody()->getLocalWaypointRec()->setX(0.0);
	p_current_waypoint.getBody()->getLocalWaypointRec()->setY(0.0);
	p_current_waypoint.getBody()->getLocalWaypointRec()->setZ(0.0);
	pEvents_ReceiveFSM->get_event_handler().set_report(QueryLocalWaypoint::ID, &p_current_waypoint);
//	this->p_pub_path.publish(path);
}

void LocalWaypointDriver_ReceiveFSM::pRosFinished(const std_msgs::Bool::ConstPtr& state)
{
	if (state->data) {
		ROS_DEBUG_NAMED("LocalWaypointDriver", "local waypoint reached");
		p_current_waypoint.getBody()->getLocalWaypointRec()->setX(0.0);
		p_current_waypoint.getBody()->getLocalWaypointRec()->setY(0.0);
		p_current_waypoint.getBody()->getLocalWaypointRec()->setZ(0.0);
		pEvents_ReceiveFSM->get_event_handler().set_report(QueryLocalWaypoint::ID, &p_current_waypoint);
	}
}

bool LocalWaypointDriver_ReceiveFSM::isControllingClient(Receive::Body::ReceiveRec transportData)
{
	//// By default, inherited guards call the parent function.
	//// This can be replaced or modified as needed.
	return pAccessControl_ReceiveFSM->isControllingClient(transportData );
}

bool LocalWaypointDriver_ReceiveFSM::waypointExists(QueryLocalWaypoint msg)
{
	/// if x, y and z are zero the waypoint is not valid
	return (p_current_waypoint.getBody()->getLocalWaypointRec()->getX() != 0
			&& p_current_waypoint.getBody()->getLocalWaypointRec()->getY() != 0
			&& p_current_waypoint.getBody()->getLocalWaypointRec()->getZ() != 0);
}

bool LocalWaypointDriver_ReceiveFSM::waypointExists(SetTravelSpeed msg)
{
	/// if x, y and z are zero the waypoint is not valid
	return (p_current_waypoint.getBody()->getLocalWaypointRec()->getX() != 0
			&& p_current_waypoint.getBody()->getLocalWaypointRec()->getY() != 0
			&& p_current_waypoint.getBody()->getLocalWaypointRec()->getZ() != 0);
}



};

