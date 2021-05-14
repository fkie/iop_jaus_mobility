

#include "urn_jaus_jss_mobility_GlobalWaypointListDriver/GlobalWaypointListDriver_ReceiveFSM.h"

#include <gps_common/conversions.h>
#include <tf/transform_datatypes.h>
#include <fkie_iop_component/iop_component.h>



using namespace JTS;

namespace urn_jaus_jss_mobility_GlobalWaypointListDriver
{



GlobalWaypointListDriver_ReceiveFSM::GlobalWaypointListDriver_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM, urn_jaus_jss_core_AccessControl::AccessControl_ReceiveFSM* pAccessControl_ReceiveFSM, urn_jaus_jss_core_Management::Management_ReceiveFSM* pManagement_ReceiveFSM, urn_jaus_jss_core_ListManager::ListManager_ReceiveFSM* pListManager_ReceiveFSM)
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new GlobalWaypointListDriver_ReceiveFSMContext(*this);

	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	this->pEvents_ReceiveFSM = pEvents_ReceiveFSM;
	this->pAccessControl_ReceiveFSM = pAccessControl_ReceiveFSM;
	this->pManagement_ReceiveFSM = pManagement_ReceiveFSM;
	this->pListManager_ReceiveFSM = pListManager_ReceiveFSM;
	p_travel_speed = 0.0;
	p_tv_max = 1.0;
	p_tf_frame_world = "/world";
	p_executing = false;
	p_last_uid = 0;
	p_current_waypoint.getBody()->getGlobalWaypointRec()->setLatitude(-90.0);
	p_current_waypoint.getBody()->getGlobalWaypointRec()->setLongitude(-180.0);
}



GlobalWaypointListDriver_ReceiveFSM::~GlobalWaypointListDriver_ReceiveFSM()
{
	delete context;
}

void GlobalWaypointListDriver_ReceiveFSM::setupNotifications()
{
	pListManager_ReceiveFSM->registerNotification("Receiving_Ready_NotControlled_Standby", ieHandler, "InternalStateChange_To_GlobalWaypointListDriver_ReceiveFSM_Receiving_Ready_NotControlled_Standby", "ListManager_ReceiveFSM");
	pListManager_ReceiveFSM->registerNotification("Receiving_Ready_NotControlled_Init", ieHandler, "InternalStateChange_To_GlobalWaypointListDriver_ReceiveFSM_Receiving_Ready_NotControlled_Init", "ListManager_ReceiveFSM");
	pListManager_ReceiveFSM->registerNotification("Receiving_Ready_NotControlled_Failure", ieHandler, "InternalStateChange_To_GlobalWaypointListDriver_ReceiveFSM_Receiving_Ready_NotControlled_Failure", "ListManager_ReceiveFSM");
	pListManager_ReceiveFSM->registerNotification("Receiving_Ready_NotControlled_Shutdown", ieHandler, "InternalStateChange_To_GlobalWaypointListDriver_ReceiveFSM_Receiving_Ready_NotControlled_Shutdown", "ListManager_ReceiveFSM");
	pListManager_ReceiveFSM->registerNotification("Receiving_Ready_NotControlled_Emergency", ieHandler, "InternalStateChange_To_GlobalWaypointListDriver_ReceiveFSM_Receiving_Ready_NotControlled_Emergency", "ListManager_ReceiveFSM");
	pListManager_ReceiveFSM->registerNotification("Receiving_Ready_NotControlled", ieHandler, "InternalStateChange_To_GlobalWaypointListDriver_ReceiveFSM_Receiving_Ready_NotControlled_Standby", "ListManager_ReceiveFSM");
	pListManager_ReceiveFSM->registerNotification("Receiving_Ready_Controlled_Standby", ieHandler, "InternalStateChange_To_GlobalWaypointListDriver_ReceiveFSM_Receiving_Ready_Controlled_Standby", "ListManager_ReceiveFSM");
	pListManager_ReceiveFSM->registerNotification("Receiving_Ready_Controlled_Ready", ieHandler, "InternalStateChange_To_GlobalWaypointListDriver_ReceiveFSM_Receiving_Ready_Controlled_Ready", "ListManager_ReceiveFSM");
	pListManager_ReceiveFSM->registerNotification("Receiving_Ready_Controlled_Failure", ieHandler, "InternalStateChange_To_GlobalWaypointListDriver_ReceiveFSM_Receiving_Ready_Controlled_Failure", "ListManager_ReceiveFSM");
	pListManager_ReceiveFSM->registerNotification("Receiving_Ready_Controlled", ieHandler, "InternalStateChange_To_GlobalWaypointListDriver_ReceiveFSM_Receiving_Ready_Controlled_Standby", "ListManager_ReceiveFSM");
	pListManager_ReceiveFSM->registerNotification("Receiving_Ready", ieHandler, "InternalStateChange_To_GlobalWaypointListDriver_ReceiveFSM_Receiving_Ready_NotControlled_Standby", "ListManager_ReceiveFSM");
	pListManager_ReceiveFSM->registerNotification("Receiving", ieHandler, "InternalStateChange_To_GlobalWaypointListDriver_ReceiveFSM_Receiving_Ready_NotControlled_Standby", "ListManager_ReceiveFSM");
	registerNotification("Receiving_Ready_NotControlled_Standby", pListManager_ReceiveFSM->getHandler(), "InternalStateChange_To_ListManager_ReceiveFSM_Receiving_Ready_NotControlled_Standby", "GlobalWaypointListDriver_ReceiveFSM");
	registerNotification("Receiving_Ready_NotControlled_Init", pListManager_ReceiveFSM->getHandler(), "InternalStateChange_To_ListManager_ReceiveFSM_Receiving_Ready_NotControlled_Init", "GlobalWaypointListDriver_ReceiveFSM");
	registerNotification("Receiving_Ready_NotControlled_Failure", pListManager_ReceiveFSM->getHandler(), "InternalStateChange_To_ListManager_ReceiveFSM_Receiving_Ready_NotControlled_Failure", "GlobalWaypointListDriver_ReceiveFSM");
	registerNotification("Receiving_Ready_NotControlled_Shutdown", pListManager_ReceiveFSM->getHandler(), "InternalStateChange_To_ListManager_ReceiveFSM_Receiving_Ready_NotControlled_Shutdown", "GlobalWaypointListDriver_ReceiveFSM");
	registerNotification("Receiving_Ready_NotControlled_Emergency", pListManager_ReceiveFSM->getHandler(), "InternalStateChange_To_ListManager_ReceiveFSM_Receiving_Ready_NotControlled_Emergency", "GlobalWaypointListDriver_ReceiveFSM");
	registerNotification("Receiving_Ready_NotControlled", pListManager_ReceiveFSM->getHandler(), "InternalStateChange_To_ListManager_ReceiveFSM_Receiving_Ready_NotControlled", "GlobalWaypointListDriver_ReceiveFSM");
	registerNotification("Receiving_Ready_Controlled_Standby", pListManager_ReceiveFSM->getHandler(), "InternalStateChange_To_ListManager_ReceiveFSM_Receiving_Ready_Controlled_Standby", "GlobalWaypointListDriver_ReceiveFSM");
	registerNotification("Receiving_Ready_Controlled_Ready", pListManager_ReceiveFSM->getHandler(), "InternalStateChange_To_ListManager_ReceiveFSM_Receiving_Ready_Controlled_Ready", "GlobalWaypointListDriver_ReceiveFSM");
	registerNotification("Receiving_Ready_Controlled_Failure", pListManager_ReceiveFSM->getHandler(), "InternalStateChange_To_ListManager_ReceiveFSM_Receiving_Ready_Controlled_Failure", "GlobalWaypointListDriver_ReceiveFSM");
	registerNotification("Receiving_Ready_Controlled", pListManager_ReceiveFSM->getHandler(), "InternalStateChange_To_ListManager_ReceiveFSM_Receiving_Ready_Controlled", "GlobalWaypointListDriver_ReceiveFSM");
	registerNotification("Receiving_Ready", pListManager_ReceiveFSM->getHandler(), "InternalStateChange_To_ListManager_ReceiveFSM_Receiving_Ready", "GlobalWaypointListDriver_ReceiveFSM");
	registerNotification("Receiving", pListManager_ReceiveFSM->getHandler(), "InternalStateChange_To_ListManager_ReceiveFSM_Receiving", "GlobalWaypointListDriver_ReceiveFSM");
	iop::Config cfg("~GlobalWaypointListDriver");
	cfg.param("tf_frame_world", p_tf_frame_world, p_tf_frame_world);
	cfg.param("tv_max", p_tv_max, p_tv_max);

	pEvents_ReceiveFSM->get_event_handler().register_query(QueryActiveElement::ID);
	pEvents_ReceiveFSM->get_event_handler().register_query(QueryGlobalWaypoint::ID);
	pListManager_ReceiveFSM->list_manager().register_supported_element(SetGlobalWaypoint::ID, this);
	//create ROS subscriber
	p_travel_speed = 0.0;
	p_pub_path = cfg.advertise<nav_msgs::Path>("cmd_global_waypoints", 5);
	p_pub_geopath = cfg.advertise<geographic_msgs::GeoPath>("cmd_global_geopath", 5);
	p_pub_tv_max = cfg.advertise<std_msgs::Float32>("cmd_travel_speed", 5);
	p_sub_finished = cfg.subscribe<std_msgs::Bool>("global_way_points_finished", 5, &GlobalWaypointListDriver_ReceiveFSM::pRosFinished, this);
	std_msgs::Float32 ros_msg;
	ros_msg.data = p_travel_speed;
	p_pub_tv_max.publish(ros_msg);
}

void GlobalWaypointListDriver_ReceiveFSM::executeWaypointListAction(ExecuteList msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress sender = transportData.getAddress();
	double speed = msg.getBody()->getExecuteListRec()->getSpeed();
	jUnsignedShortInteger start_iud = msg.getBody()->getExecuteListRec()->getElementUID();
	ROS_DEBUG_NAMED("GlobalWaypointListDriver", "execute waypoint list with elements: %d, speed %.2f, start uid: %d", pListManager_ReceiveFSM->list_manager().size(), speed, start_iud);
	if (!pListManager_ReceiveFSM->list_manager().execute_list(start_iud, speed)) {
		ROS_WARN_NAMED("ListManager", "execute waypoint list failed with error: %d (%s)", pListManager_ReceiveFSM->list_manager().get_error_code(), pListManager_ReceiveFSM->list_manager().get_error_msg().c_str());
		RejectElementRequest reject;
		reject.getBody()->getRejectElementRec()->setRequestID(0);
		reject.getBody()->getRejectElementRec()->setResponseCode(pListManager_ReceiveFSM->list_manager().get_error_code());
		sendJausMessage(reject, sender);
	}
}

void GlobalWaypointListDriver_ReceiveFSM::modifyTravelSpeedAction(ExecuteList msg)
{
	double speed = msg.getBody()->getExecuteListRec()->getSpeed();
	ROS_DEBUG_NAMED("GlobalWaypointListDriver", "modify travel speed to %.2f", speed);
	p_travel_speed = speed;
	if (p_travel_speed > p_tv_max) {
		ROS_DEBUG_NAMED("GlobalWaypointListDriver", "  reset travel speed to max %.2f", p_tv_max);
		p_travel_speed = p_tv_max;
	}
	std_msgs::Float32 ros_msg;
	ros_msg.data = p_travel_speed;
	p_pub_tv_max.publish(ros_msg);
}

void GlobalWaypointListDriver_ReceiveFSM::resetTravelSpeedAction()
{
	ROS_DEBUG_NAMED("GlobalWaypointListDriver", "reset travel speed to default: %.2f", 0.0);
	p_travel_speed = 0.0;
	std_msgs::Float32 ros_msg;
	ros_msg.data = p_travel_speed;
	p_pub_tv_max.publish(ros_msg);
}

void GlobalWaypointListDriver_ReceiveFSM::sendReportActiveElementAction(QueryActiveElement msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress requester = transportData.getAddress();
	jUnsignedShortInteger cuid = pListManager_ReceiveFSM->list_manager().get_current_element();
	ROS_DEBUG_NAMED("GlobalWaypointListDriver", "report active element %d to %s", cuid, requester.str().c_str());
	ReportActiveElement reply;
	reply.getBody()->getActiveElementRec()->setElementUID(cuid);
	sendJausMessage(reply, requester);
}

void GlobalWaypointListDriver_ReceiveFSM::sendReportGlobalWaypointAction(QueryGlobalWaypoint msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress requester = transportData.getAddress();
	jUnsignedShortInteger cuid = pListManager_ReceiveFSM->list_manager().get_current_element();
	if (cuid != 65535 && cuid != 0) {
		ROS_DEBUG_NAMED("GlobalWaypointListDriver", "report current global point (uid: %d) to %s", cuid, requester.str().c_str());
		iop::InternalElement el = pListManager_ReceiveFSM->list_manager().get_element(cuid);
		if (el.get_uid() != 0) {
			ReportGlobalWaypoint reply;
			reply.decode(el.get_report().getBody()->getElementRec()->getElementData()->getData());
			ROS_DEBUG_NAMED("GlobalWaypointListDriver", "  global waypoint: lat %.2f, lon %.2f",
					reply.getBody()->getGlobalWaypointRec()->getLatitude(), reply.getBody()->getGlobalWaypointRec()->getLongitude());
			sendJausMessage(reply, requester);
		}
	}
	// perhaps there is also GlobalWaypointDriver and it can handle this message
	throw std::exception();
}

void GlobalWaypointListDriver_ReceiveFSM::sendReportTravelSpeedAction(QueryTravelSpeed msg, Receive::Body::ReceiveRec transportData)
{
	jUnsignedShortInteger cuid = pListManager_ReceiveFSM->list_manager().get_current_element();
	if (cuid != 65535 && cuid != 0) {
		JausAddress requester = transportData.getAddress();
		ROS_DEBUG_NAMED("GlobalWaypointListDriver", "report current travel speed %.2f to %s", p_travel_speed, requester.str().c_str());
		ReportTravelSpeed reply;
		reply.getBody()->getTravelSpeedRec()->setSpeed(p_travel_speed);
		sendJausMessage(reply, requester);
	}
	// perhaps there is also GlobalWaypointDriver and it can handle this message
	throw std::exception();
}



bool GlobalWaypointListDriver_ReceiveFSM::elementExists(ExecuteList msg)
{
	jUnsignedShortInteger uid = msg.getBody()->getExecuteListRec()->getElementUID();
	if (uid == 0) {
		return (pListManager_ReceiveFSM->list_manager().size() > 0);
	}
	return pListManager_ReceiveFSM->list_manager().elementExists(msg.getBody()->getExecuteListRec()->getElementUID());
}

bool GlobalWaypointListDriver_ReceiveFSM::elementSpecified(ExecuteList msg)
{
	jUnsignedShortInteger uid = msg.getBody()->getExecuteListRec()->getElementUID();
	return (uid != 0 && uid != 65535);
}

bool GlobalWaypointListDriver_ReceiveFSM::isControllingClient(Receive::Body::ReceiveRec transportData)
{
	//// By default, inherited guards call the parent function.
	//// This can be replaced or modified as needed.
	return pAccessControl_ReceiveFSM->isControllingClient(transportData );
}


void GlobalWaypointListDriver_ReceiveFSM::execute_list(std::vector<iop::InternalElement> elements, double speed)
{
	if (!isnan(speed)) {
		p_travel_speed = speed;
		if (p_travel_speed > p_tv_max) {
			p_travel_speed = p_tv_max;
		}
		std_msgs::Float32 ros_msg;
		ros_msg.data = p_travel_speed;
		p_pub_tv_max.publish(ros_msg);
	}
	nav_msgs::Path path;
	path.header.stamp = ros::Time::now();
	path.header.frame_id = p_tf_frame_world;
	geographic_msgs::GeoPath geopath;
	geopath.header = path.header;
	std::vector<iop::InternalElement>::iterator it;
	if (elements.size() == 0) {
		p_current_element.getBody()->getActiveElementRec()->setElementUID(0);
		pEvents_ReceiveFSM->get_event_handler().set_report(QueryActiveElement::ID, &p_current_element);
		ReportGlobalWaypoint waypoint;
		p_current_waypoint = waypoint;
		p_current_waypoint.getBody()->getGlobalWaypointRec()->setLatitude(-90.0);
		p_current_waypoint.getBody()->getGlobalWaypointRec()->setLongitude(-180.0);
		pEvents_ReceiveFSM->get_event_handler().set_report(QueryGlobalWaypoint::ID, &p_current_waypoint);
		stop_execution();
	} else {
		p_executing = true;
	}
	for (it = elements.begin(); it != elements.end(); ++it) {
		geometry_msgs::PoseStamped pose = get_pose_from_waypoint(*it, it == elements.begin());
		if (it == elements.begin()) {
			pListManager_ReceiveFSM->list_manager().set_current_element(it->get_uid());
			p_current_element.getBody()->getActiveElementRec()->setElementUID(it->get_uid());
			pEvents_ReceiveFSM->get_event_handler().set_report(QueryActiveElement::ID, &p_current_element);
			pEvents_ReceiveFSM->get_event_handler().set_report(QueryGlobalWaypoint::ID, &p_current_waypoint);
		}
		pose.header = path.header;
		path.poses.push_back(pose);
		geographic_msgs::GeoPoseStamped geopose = get_geopose_from_waypoint(*it, false);
		geopose.header = geopath.header;
		geopath.poses.push_back(geopose);
		p_last_uid = it->get_uid();
	}
	p_pub_path.publish(path);
	p_pub_geopath.publish(geopath);
}

void GlobalWaypointListDriver_ReceiveFSM::stop_execution()
{
	if (p_executing) {
		nav_msgs::Path path;
		path.header.stamp = ros::Time::now();
		p_pub_path.publish(path);
		ReportGlobalWaypoint waypoint;
		p_current_waypoint = waypoint;
		p_current_waypoint.getBody()->getGlobalWaypointRec()->setLatitude(-90.0);
		p_current_waypoint.getBody()->getGlobalWaypointRec()->setLongitude(-180.0);
		pEvents_ReceiveFSM->get_event_handler().set_report(QueryGlobalWaypoint::ID, &p_current_waypoint);
		p_last_uid = 0;
		p_executing = false;
		p_current_element.getBody()->getActiveElementRec()->setElementUID(0);
		pEvents_ReceiveFSM->get_event_handler().set_report(QueryActiveElement::ID, &p_current_element);
	}
}

void GlobalWaypointListDriver_ReceiveFSM::pRosFinished(const std_msgs::Bool::ConstPtr& state)
{
	if (state->data && p_last_uid != 0) {
		ROS_DEBUG_NAMED("GlobalWaypointListDriver", "execution finished");
		bool finished = pListManager_ReceiveFSM->list_manager().finished(p_last_uid);
		if (finished) {
			ROS_DEBUG_NAMED("GlobalWaypointListDriver", "  there are futher elements available, execute!");
		} else {
			stop_execution();
		}
	}
}

geometry_msgs::PoseStamped GlobalWaypointListDriver_ReceiveFSM::get_pose_from_waypoint(iop::InternalElement& element, bool update_current)
{
	SetGlobalWaypoint wp;
	wp.decode(element.get_report().getBody()->getElementRec()->getElementData()->getData());
	geometry_msgs::PoseStamped result;
	double lat, lon, alt = 0.0;
	double roll, pitch, yaw = 0.0;
	SetGlobalWaypoint::Body::GlobalWaypointRec *wprec = wp.getBody()->getGlobalWaypointRec();
	lat = wprec->getLatitude();
	lon = wprec->getLongitude();
	if (wprec->isAltitudeValid()) {
		alt = wprec->getAltitude();
	}
	if (wprec->isRollValid()) {
		roll = wprec->getRoll();
	}
	if (wprec->isPitchValid()) {
		pitch = wprec->getPitch();
	}
	if (wprec->isYawValid()) {
		yaw = wprec->getYaw();
	}
	if (update_current) {
		p_current_waypoint.getBody()->getGlobalWaypointRec()->setLatitude(lat);
		p_current_waypoint.getBody()->getGlobalWaypointRec()->setLongitude(lon);
		if (wprec->isAltitudeValid()) {
			p_current_waypoint.getBody()->getGlobalWaypointRec()->setAltitude(alt);
		}
		if (wprec->isRollValid()) {
			p_current_waypoint.getBody()->getGlobalWaypointRec()->setRoll(roll);
		}
		if (wprec->isPitchValid()) {
			p_current_waypoint.getBody()->getGlobalWaypointRec()->setPitch(pitch);
		}
		if (wprec->isYawValid()) {
			p_current_waypoint.getBody()->getGlobalWaypointRec()->setYaw(yaw);
		}
		if (wprec->isWaypointToleranceValid()) {
			p_current_waypoint.getBody()->getGlobalWaypointRec()->setWaypointTolerance(wprec->getWaypointTolerance());
		}
		pEvents_ReceiveFSM->get_event_handler().set_report(QueryGlobalWaypoint::ID, &p_current_waypoint);
	}
	double northing, easting;
	std::string zone;
	gps_common::LLtoUTM(lat, lon, northing, easting, zone);
	tf::Quaternion quat = tf::createQuaternionFromRPY(roll, pitch, yaw);

	if (lat > -90.0 && lon > -180.0) {
		ROS_DEBUG_NAMED("GlobalWaypointListDriver", "add Waypoint lat: %.6f, lon: %.6f, alt: %.2f, roll: %.2f, pitch: %.2f, yaw: %.2f", lat, lon, alt, roll, pitch, yaw);
		result.pose.position.x = easting;
		result.pose.position.y = northing;
		result.pose.position.z = alt;
		result.pose.orientation.x = quat.x();
		result.pose.orientation.y = quat.y();
		result.pose.orientation.z = quat.z();
		result.pose.orientation.w = quat.w();
	}
	return result;
}

geographic_msgs::GeoPoseStamped GlobalWaypointListDriver_ReceiveFSM::get_geopose_from_waypoint(iop::InternalElement& element, bool update_current)
{
	SetGlobalWaypoint wp;
	wp.decode(element.get_report().getBody()->getElementRec()->getElementData()->getData());
	geographic_msgs::GeoPoseStamped result;
	double lat, lon, alt = 0.0;
	double roll, pitch, yaw = 0.0;
	SetGlobalWaypoint::Body::GlobalWaypointRec *wprec = wp.getBody()->getGlobalWaypointRec();
	lat = wprec->getLatitude();
	lon = wprec->getLongitude();
	if (wprec->isAltitudeValid()) {
		alt = wprec->getAltitude();
	}
	if (wprec->isRollValid()) {
		roll = wprec->getRoll();
	}
	if (wprec->isPitchValid()) {
		pitch = wprec->getPitch();
	}
	if (wprec->isYawValid()) {
		yaw = wprec->getYaw();
	}
	if (update_current) {
		p_current_waypoint.getBody()->getGlobalWaypointRec()->setLatitude(lat);
		p_current_waypoint.getBody()->getGlobalWaypointRec()->setLongitude(lon);
		if (wprec->isAltitudeValid()) {
			p_current_waypoint.getBody()->getGlobalWaypointRec()->setAltitude(alt);
		}
		if (wprec->isRollValid()) {
			p_current_waypoint.getBody()->getGlobalWaypointRec()->setRoll(roll);
		}
		if (wprec->isPitchValid()) {
			p_current_waypoint.getBody()->getGlobalWaypointRec()->setPitch(pitch);
		}
		if (wprec->isYawValid()) {
			p_current_waypoint.getBody()->getGlobalWaypointRec()->setYaw(yaw);
		}
		if (wprec->isWaypointToleranceValid()) {
			p_current_waypoint.getBody()->getGlobalWaypointRec()->setWaypointTolerance(wprec->getWaypointTolerance());
		}
		pEvents_ReceiveFSM->get_event_handler().set_report(QueryGlobalWaypoint::ID, &p_current_waypoint);
	}
	tf::Quaternion quat = tf::createQuaternionFromRPY(roll, pitch, yaw);

	if (lat > -90.0 && lon > -180.0) {
		ROS_DEBUG_NAMED("GlobalWaypointListDriver", "add Waypoint lat: %.6f, lon: %.6f, alt: %.2f, roll: %.2f, pitch: %.2f, yaw: %.2f", lat, lon, alt, roll, pitch, yaw);
		result.pose.position.latitude = lat;
		result.pose.position.longitude = lon;
		result.pose.position.altitude = alt;
		result.pose.orientation.x = quat.x();
		result.pose.orientation.y = quat.y();
		result.pose.orientation.z = quat.z();
		result.pose.orientation.w = quat.w();
	}
	return result;

}

};
