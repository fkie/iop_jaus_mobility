

#include "urn_jaus_jss_mobility_LocalWaypointListDriver/LocalWaypointListDriver_ReceiveFSM.h"
#include <fkie_iop_component/iop_config.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>



using namespace JTS;

namespace urn_jaus_jss_mobility_LocalWaypointListDriver
{



LocalWaypointListDriver_ReceiveFSM::LocalWaypointListDriver_ReceiveFSM(std::shared_ptr<iop::Component> cmp, urn_jaus_jss_core_ListManager::ListManager_ReceiveFSM* pListManager_ReceiveFSM, urn_jaus_jss_core_Management::Management_ReceiveFSM* pManagement_ReceiveFSM, urn_jaus_jss_core_AccessControl::AccessControl_ReceiveFSM* pAccessControl_ReceiveFSM, urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM, urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM)
: logger(cmp->get_logger().get_child("LocalWaypointListDriver"))
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new LocalWaypointListDriver_ReceiveFSMContext(*this);

	this->pListManager_ReceiveFSM = pListManager_ReceiveFSM;
	this->pManagement_ReceiveFSM = pManagement_ReceiveFSM;
	this->pAccessControl_ReceiveFSM = pAccessControl_ReceiveFSM;
	this->pEvents_ReceiveFSM = pEvents_ReceiveFSM;
	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	this->cmp = cmp;
	p_travel_speed = 0.0;
	p_tv_max = 1.0;
	p_tf_frame_robot = "base_link";
	p_executing = false;
	p_last_uid = 0;
	p_current_waypoint.getBody()->getLocalWaypointRec()->setX(0);
	p_current_waypoint.getBody()->getLocalWaypointRec()->setY(0);
}



LocalWaypointListDriver_ReceiveFSM::~LocalWaypointListDriver_ReceiveFSM()
{
	delete context;
}

void LocalWaypointListDriver_ReceiveFSM::setupNotifications()
{
	pListManager_ReceiveFSM->registerNotification("Receiving_Ready_NotControlled_Standby", ieHandler, "InternalStateChange_To_LocalWaypointListDriver_ReceiveFSM_Receiving_Ready_NotControlled_Standby", "ListManager_ReceiveFSM");
	pListManager_ReceiveFSM->registerNotification("Receiving_Ready_NotControlled_Init", ieHandler, "InternalStateChange_To_LocalWaypointListDriver_ReceiveFSM_Receiving_Ready_NotControlled_Init", "ListManager_ReceiveFSM");
	pListManager_ReceiveFSM->registerNotification("Receiving_Ready_NotControlled_Failure", ieHandler, "InternalStateChange_To_LocalWaypointListDriver_ReceiveFSM_Receiving_Ready_NotControlled_Failure", "ListManager_ReceiveFSM");
	pListManager_ReceiveFSM->registerNotification("Receiving_Ready_NotControlled_Shutdown", ieHandler, "InternalStateChange_To_LocalWaypointListDriver_ReceiveFSM_Receiving_Ready_NotControlled_Shutdown", "ListManager_ReceiveFSM");
	pListManager_ReceiveFSM->registerNotification("Receiving_Ready_NotControlled_Emergency", ieHandler, "InternalStateChange_To_LocalWaypointListDriver_ReceiveFSM_Receiving_Ready_NotControlled_Emergency", "ListManager_ReceiveFSM");
	pListManager_ReceiveFSM->registerNotification("Receiving_Ready_NotControlled", ieHandler, "InternalStateChange_To_LocalWaypointListDriver_ReceiveFSM_Receiving_Ready_NotControlled_Standby", "ListManager_ReceiveFSM");
	pListManager_ReceiveFSM->registerNotification("Receiving_Ready_Controlled_Standby", ieHandler, "InternalStateChange_To_LocalWaypointListDriver_ReceiveFSM_Receiving_Ready_Controlled_Standby", "ListManager_ReceiveFSM");
	pListManager_ReceiveFSM->registerNotification("Receiving_Ready_Controlled_Ready", ieHandler, "InternalStateChange_To_LocalWaypointListDriver_ReceiveFSM_Receiving_Ready_Controlled_Ready", "ListManager_ReceiveFSM");
	pListManager_ReceiveFSM->registerNotification("Receiving_Ready_Controlled_Failure", ieHandler, "InternalStateChange_To_LocalWaypointListDriver_ReceiveFSM_Receiving_Ready_Controlled_Failure", "ListManager_ReceiveFSM");
	pListManager_ReceiveFSM->registerNotification("Receiving_Ready_Controlled", ieHandler, "InternalStateChange_To_LocalWaypointListDriver_ReceiveFSM_Receiving_Ready_Controlled_Standby", "ListManager_ReceiveFSM");
	pListManager_ReceiveFSM->registerNotification("Receiving_Ready", ieHandler, "InternalStateChange_To_LocalWaypointListDriver_ReceiveFSM_Receiving_Ready_NotControlled_Standby", "ListManager_ReceiveFSM");
	pListManager_ReceiveFSM->registerNotification("Receiving", ieHandler, "InternalStateChange_To_LocalWaypointListDriver_ReceiveFSM_Receiving_Ready_NotControlled_Standby", "ListManager_ReceiveFSM");
	registerNotification("Receiving_Ready_NotControlled_Standby", pListManager_ReceiveFSM->getHandler(), "InternalStateChange_To_ListManager_ReceiveFSM_Receiving_Ready_NotControlled_Standby", "LocalWaypointListDriver_ReceiveFSM");
	registerNotification("Receiving_Ready_NotControlled_Init", pListManager_ReceiveFSM->getHandler(), "InternalStateChange_To_ListManager_ReceiveFSM_Receiving_Ready_NotControlled_Init", "LocalWaypointListDriver_ReceiveFSM");
	registerNotification("Receiving_Ready_NotControlled_Failure", pListManager_ReceiveFSM->getHandler(), "InternalStateChange_To_ListManager_ReceiveFSM_Receiving_Ready_NotControlled_Failure", "LocalWaypointListDriver_ReceiveFSM");
	registerNotification("Receiving_Ready_NotControlled_Shutdown", pListManager_ReceiveFSM->getHandler(), "InternalStateChange_To_ListManager_ReceiveFSM_Receiving_Ready_NotControlled_Shutdown", "LocalWaypointListDriver_ReceiveFSM");
	registerNotification("Receiving_Ready_NotControlled_Emergency", pListManager_ReceiveFSM->getHandler(), "InternalStateChange_To_ListManager_ReceiveFSM_Receiving_Ready_NotControlled_Emergency", "LocalWaypointListDriver_ReceiveFSM");
	registerNotification("Receiving_Ready_NotControlled", pListManager_ReceiveFSM->getHandler(), "InternalStateChange_To_ListManager_ReceiveFSM_Receiving_Ready_NotControlled", "LocalWaypointListDriver_ReceiveFSM");
	registerNotification("Receiving_Ready_Controlled_Standby", pListManager_ReceiveFSM->getHandler(), "InternalStateChange_To_ListManager_ReceiveFSM_Receiving_Ready_Controlled_Standby", "LocalWaypointListDriver_ReceiveFSM");
	registerNotification("Receiving_Ready_Controlled_Ready", pListManager_ReceiveFSM->getHandler(), "InternalStateChange_To_ListManager_ReceiveFSM_Receiving_Ready_Controlled_Ready", "LocalWaypointListDriver_ReceiveFSM");
	registerNotification("Receiving_Ready_Controlled_Failure", pListManager_ReceiveFSM->getHandler(), "InternalStateChange_To_ListManager_ReceiveFSM_Receiving_Ready_Controlled_Failure", "LocalWaypointListDriver_ReceiveFSM");
	registerNotification("Receiving_Ready_Controlled", pListManager_ReceiveFSM->getHandler(), "InternalStateChange_To_ListManager_ReceiveFSM_Receiving_Ready_Controlled", "LocalWaypointListDriver_ReceiveFSM");
	registerNotification("Receiving_Ready", pListManager_ReceiveFSM->getHandler(), "InternalStateChange_To_ListManager_ReceiveFSM_Receiving_Ready", "LocalWaypointListDriver_ReceiveFSM");
	registerNotification("Receiving", pListManager_ReceiveFSM->getHandler(), "InternalStateChange_To_ListManager_ReceiveFSM_Receiving", "LocalWaypointListDriver_ReceiveFSM");
}


void LocalWaypointListDriver_ReceiveFSM::setupIopConfiguration()
{
	iop::Config cfg(cmp, "LocalWaypointListDriver");
	cfg.declare_param<std::string>("tf_frame_robot", p_tf_frame_robot, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_STRING,
		"TF frame used in ROS for local coordinates. This value is set in each command message.",
		"Default: 'base_link'");
	cfg.declare_param<double>("tv_max", p_tv_max, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE,
		"The maximum allowed speed.",
		"Default: 1.0");
	cfg.param("tf_frame_robot", p_tf_frame_robot, p_tf_frame_robot);
	cfg.param("tv_max", p_tv_max, p_tv_max);

	pEvents_ReceiveFSM->get_event_handler().register_query(QueryActiveElement::ID);
	pEvents_ReceiveFSM->get_event_handler().register_query(QueryLocalWaypoint::ID);
	pListManager_ReceiveFSM->list_manager().register_supported_element(SetLocalWaypoint::ID, this);
	//create ROS subscriber
	p_travel_speed = 0.0;
	p_pub_path = cfg.create_publisher<nav_msgs::msg::Path>("cmd_local_waypoints", 5);
	p_pub_pose = cfg.create_publisher<geometry_msgs::msg::PoseStamped>("cmd_local_pose", 5);
	p_pub_tv_max = cfg.create_publisher<std_msgs::msg::Float32>("cmd_travel_speed", 5);
	p_sub_finished = cfg.create_subscription<std_msgs::msg::Bool>("local_way_points_finished", 5, std::bind(&LocalWaypointListDriver_ReceiveFSM::pRosFinished, this, std::placeholders::_1));
	auto ros_msg = std_msgs::msg::Float32();
	ros_msg.data = p_travel_speed;
	p_pub_tv_max->publish(ros_msg);
}

void LocalWaypointListDriver_ReceiveFSM::executeWaypointListAction(ExecuteList msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress sender = transportData.getAddress();
	double speed = msg.getBody()->getExecuteListRec()->getSpeed();
	jUnsignedShortInteger start_iud = msg.getBody()->getExecuteListRec()->getElementUID();
	RCLCPP_DEBUG(logger, "execute waypoint list with elements: %d, speed %.2f, start uid: %d", pListManager_ReceiveFSM->list_manager().size(), speed, start_iud);
	if (!pListManager_ReceiveFSM->list_manager().execute_list(start_iud, speed)) {
		RCLCPP_WARN(logger, "execute waypoint list failed with error: %d (%s)", pListManager_ReceiveFSM->list_manager().get_error_code(), pListManager_ReceiveFSM->list_manager().get_error_msg().c_str());
		RejectElementRequest reject;
		reject.getBody()->getRejectElementRec()->setRequestID(0);
		reject.getBody()->getRejectElementRec()->setResponseCode(pListManager_ReceiveFSM->list_manager().get_error_code());
		sendJausMessage(reject, sender);
	}
}

void LocalWaypointListDriver_ReceiveFSM::modifyTravelSpeedAction(ExecuteList msg)
{
	double speed = msg.getBody()->getExecuteListRec()->getSpeed();
	RCLCPP_DEBUG(logger, "modify travel speed to %.2f", speed);
	p_travel_speed = speed;
	if (p_travel_speed > p_tv_max) {
		RCLCPP_DEBUG(logger, "  reset travel speed to max %.2f", p_tv_max);
		p_travel_speed = p_tv_max;
	}
	auto ros_msg = std_msgs::msg::Float32();
	ros_msg.data = p_travel_speed;
	p_pub_tv_max->publish(ros_msg);
}

void LocalWaypointListDriver_ReceiveFSM::resetTravelSpeedAction()
{
	RCLCPP_DEBUG(logger, "reset travel speed to default: %.2f", 0.0);
	p_travel_speed = 0.0;
	auto ros_msg = std_msgs::msg::Float32();
	ros_msg.data = p_travel_speed;
	p_pub_tv_max->publish(ros_msg);
}

void LocalWaypointListDriver_ReceiveFSM::sendReportActiveElementAction(QueryActiveElement msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress requester = transportData.getAddress();
	jUnsignedShortInteger cuid = pListManager_ReceiveFSM->list_manager().get_current_element();
	RCLCPP_DEBUG(logger, "report active element %d to %s", cuid, requester.str().c_str());
	ReportActiveElement reply;
	reply.getBody()->getActiveElementRec()->setElementUID(cuid);
	sendJausMessage(reply, requester);
}

void LocalWaypointListDriver_ReceiveFSM::sendReportLocalWaypointAction(QueryLocalWaypoint msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress requester = transportData.getAddress();
	jUnsignedShortInteger cuid = pListManager_ReceiveFSM->list_manager().get_current_element();
	if (cuid != 65535 && cuid != 0) {
		RCLCPP_DEBUG(logger, "report current local point (uid: %d) to %s", cuid, requester.str().c_str());
		iop::InternalElement el = pListManager_ReceiveFSM->list_manager().get_element(cuid);
		if (el.get_uid() != 0) {
			ReportLocalWaypoint reply;
			reply.decode(el.get_report().getBody()->getElementRec()->getElementData()->getData());
			RCLCPP_DEBUG(logger, "  local waypoint: x %.2f, y %.2f",
					reply.getBody()->getLocalWaypointRec()->getX(), reply.getBody()->getLocalWaypointRec()->getY());
			sendJausMessage(reply, requester);
		}
	}
	// perhaps there is also LocalWaypointDriver and it can handle this message
	throw std::exception();
}

void LocalWaypointListDriver_ReceiveFSM::sendReportTravelSpeedAction(QueryTravelSpeed msg, Receive::Body::ReceiveRec transportData)
{
	jUnsignedShortInteger cuid = pListManager_ReceiveFSM->list_manager().get_current_element();
	if (cuid != 65535 && cuid != 0) {
		JausAddress requester = transportData.getAddress();
		RCLCPP_DEBUG(logger, "report current travel speed %.2f to %s", p_travel_speed, requester.str().c_str());
		ReportTravelSpeed reply;
		reply.getBody()->getTravelSpeedRec()->setSpeed(p_travel_speed);
		sendJausMessage(reply, requester);
	}
	// perhaps there is also LocalWaypointDriver and it can handle this message
	throw std::exception();
}



bool LocalWaypointListDriver_ReceiveFSM::elementExists(ExecuteList msg)
{
	jUnsignedShortInteger uid = msg.getBody()->getExecuteListRec()->getElementUID();
	if (uid == 0) {
		return (pListManager_ReceiveFSM->list_manager().size() > 0);
	}
	return pListManager_ReceiveFSM->list_manager().elementExists(msg.getBody()->getExecuteListRec()->getElementUID());
}

bool LocalWaypointListDriver_ReceiveFSM::elementSpecified(ExecuteList msg)
{
	jUnsignedShortInteger uid = msg.getBody()->getExecuteListRec()->getElementUID();
	return (uid != 0 && uid != 65535);
}

bool LocalWaypointListDriver_ReceiveFSM::isControllingClient(Receive::Body::ReceiveRec transportData)
{
	//// By default, inherited guards call the parent function.
	//// This can be replaced or modified as needed.
	return pAccessControl_ReceiveFSM->isControllingClient(transportData );
}


void LocalWaypointListDriver_ReceiveFSM::execute_list(std::vector<iop::InternalElement> elements, double speed)
{
	if (!isnan(speed)) {
		p_travel_speed = speed;
		if (p_travel_speed > p_tv_max) {
			p_travel_speed = p_tv_max;
		}
		auto ros_msg = std_msgs::msg::Float32();
		ros_msg.data = p_travel_speed;
		p_pub_tv_max->publish(ros_msg);
	}
	auto path = nav_msgs::msg::Path();
	path.header.stamp = cmp->now();
	path.header.frame_id = p_tf_frame_robot;
	std::vector<iop::InternalElement>::iterator it;
	if (elements.size() == 0) {
		p_current_element.getBody()->getActiveElementRec()->setElementUID(0);
		pEvents_ReceiveFSM->get_event_handler().set_report(QueryActiveElement::ID, &p_current_element);
		ReportLocalWaypoint waypoint;
		p_current_waypoint = waypoint;
		p_current_waypoint.getBody()->getLocalWaypointRec()->setX(0.0);
		p_current_waypoint.getBody()->getLocalWaypointRec()->setY(0.0);
		p_current_waypoint.getBody()->getLocalWaypointRec()->setZ(0.0);
		pEvents_ReceiveFSM->get_event_handler().set_report(QueryLocalWaypoint::ID, &p_current_waypoint);
		stop_execution();
	} else {
		p_executing = true;
	}
	for (it = elements.begin(); it != elements.end(); ++it) {
		geometry_msgs::msg::PoseStamped pose = get_pose_from_waypoint(*it, it == elements.begin());
		pose.header = path.header;
		if (it == elements.begin()) {
			pListManager_ReceiveFSM->list_manager().set_current_element(it->get_uid());
			p_current_element.getBody()->getActiveElementRec()->setElementUID(it->get_uid());
			pEvents_ReceiveFSM->get_event_handler().set_report(QueryActiveElement::ID, &p_current_element);
			pEvents_ReceiveFSM->get_event_handler().set_report(QueryLocalWaypoint::ID, &p_current_waypoint);
			p_pub_pose->publish(pose);
		}
		path.poses.push_back(pose);
		p_last_uid = it->get_uid();
	}
	RCLCPP_INFO(logger, "publish command path with %d waypoints", path.poses.size());
	p_pub_path->publish(path);
}

void LocalWaypointListDriver_ReceiveFSM::stop_execution()
{
	if (p_executing) {
		auto path = nav_msgs::msg::Path();
		path.header.stamp = cmp->now();
		p_pub_path->publish(path);
		ReportLocalWaypoint waypoint;
		p_current_waypoint = waypoint;
		p_current_waypoint.getBody()->getLocalWaypointRec()->setX(0.0);
		p_current_waypoint.getBody()->getLocalWaypointRec()->setY(0.0);
		p_current_waypoint.getBody()->getLocalWaypointRec()->setZ(0.0);
		pEvents_ReceiveFSM->get_event_handler().set_report(QueryLocalWaypoint::ID, &p_current_waypoint);
		p_last_uid = 0;
		p_executing = false;
		p_current_element.getBody()->getActiveElementRec()->setElementUID(0);
		pEvents_ReceiveFSM->get_event_handler().set_report(QueryActiveElement::ID, &p_current_element);
	}
}

void LocalWaypointListDriver_ReceiveFSM::pRosFinished(const std_msgs::msg::Bool::SharedPtr state)
{
	if (state->data && p_last_uid != 0) {
		RCLCPP_DEBUG(logger, "execution finished");
		bool finished = pListManager_ReceiveFSM->list_manager().finished(p_last_uid);
		if (finished) {
			RCLCPP_DEBUG(logger, "  there are futher elements available, execute!");
		} else {
			stop_execution();
		}
	}
}

geometry_msgs::msg::PoseStamped LocalWaypointListDriver_ReceiveFSM::get_pose_from_waypoint(iop::InternalElement& element, bool update_current)
{
	SetLocalWaypoint wp;
	wp.decode(element.get_report().getBody()->getElementRec()->getElementData()->getData());
	auto result = geometry_msgs::msg::PoseStamped();
	double x, y, z = 0.0;
	double roll, pitch, yaw = 0.0;
	SetLocalWaypoint::Body::LocalWaypointRec *wprec = wp.getBody()->getLocalWaypointRec();
	x = wprec->getX();
	y = wprec->getY();
	if (wprec->isZValid()) {
		z = wprec->getZ();
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
		p_current_waypoint.getBody()->getLocalWaypointRec()->setX(x);
		p_current_waypoint.getBody()->getLocalWaypointRec()->setY(y);
		if (wprec->isZValid()) {
			p_current_waypoint.getBody()->getLocalWaypointRec()->setZ(z);
		}
		if (wprec->isRollValid()) {
			p_current_waypoint.getBody()->getLocalWaypointRec()->setRoll(roll);
		}
		if (wprec->isPitchValid()) {
			p_current_waypoint.getBody()->getLocalWaypointRec()->setPitch(pitch);
		}
		if (wprec->isYawValid()) {
			p_current_waypoint.getBody()->getLocalWaypointRec()->setYaw(yaw);
		}
		if (wprec->isWaypointToleranceValid()) {
			p_current_waypoint.getBody()->getLocalWaypointRec()->setWaypointTolerance(wprec->getWaypointTolerance());
		}
		pEvents_ReceiveFSM->get_event_handler().set_report(QueryLocalWaypoint::ID, &p_current_waypoint);
	}
	tf2::Quaternion quat;
	quat.setRPY(roll, pitch, yaw);

	RCLCPP_DEBUG(logger, "add Waypoint x: %.6f, y: %.6f, z: %.2f, roll: %.2f, pitch: %.2f, yaw: %.2f", x, y, z, roll, pitch, yaw);
	result.pose.position.x = x;
	result.pose.position.y = y;
	result.pose.position.z = z;
	result.pose.orientation.x = quat.x();
	result.pose.orientation.y = quat.y();
	result.pose.orientation.z = quat.z();
	result.pose.orientation.w = quat.w();

	return result;
}

}
