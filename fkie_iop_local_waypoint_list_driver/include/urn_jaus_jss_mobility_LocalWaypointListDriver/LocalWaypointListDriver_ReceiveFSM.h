

#ifndef LOCALWAYPOINTLISTDRIVER_RECEIVEFSM_H
#define LOCALWAYPOINTLISTDRIVER_RECEIVEFSM_H

#include "JausUtils.h"
#include "InternalEvents/InternalEventHandler.h"
#include "Transport/JausTransport.h"
#include "JTSStateMachine.h"
#include "urn_jaus_jss_mobility_LocalWaypointListDriver/Messages/MessageSet.h"
#include "urn_jaus_jss_mobility_LocalWaypointListDriver/InternalEvents/InternalEventsSet.h"

#include "InternalEvents/Receive.h"
#include "InternalEvents/Send.h"

#include "urn_jaus_jss_core_ListManager/ListManager_ReceiveFSM.h"
#include "urn_jaus_jss_core_Management/Management_ReceiveFSM.h"
#include "urn_jaus_jss_core_AccessControl/AccessControl_ReceiveFSM.h"
#include "urn_jaus_jss_core_Events/Events_ReceiveFSM.h"
#include "urn_jaus_jss_core_Transport/Transport_ReceiveFSM.h"


#include "LocalWaypointListDriver_ReceiveFSM_sm.h"
#include <rclcpp/rclcpp.hpp>
#include <fkie_iop_component/iop_component.hpp>

#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <fkie_iop_list_manager/ListManagerListenerInterface.h>

namespace urn_jaus_jss_mobility_LocalWaypointListDriver
{

class DllExport LocalWaypointListDriver_ReceiveFSM : public JTS::StateMachine, public iop::ListManagerListenerInterface
{
public:
	LocalWaypointListDriver_ReceiveFSM(std::shared_ptr<iop::Component> cmp, urn_jaus_jss_core_ListManager::ListManager_ReceiveFSM* pListManager_ReceiveFSM, urn_jaus_jss_core_Management::Management_ReceiveFSM* pManagement_ReceiveFSM, urn_jaus_jss_core_AccessControl::AccessControl_ReceiveFSM* pAccessControl_ReceiveFSM, urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM, urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM);
	virtual ~LocalWaypointListDriver_ReceiveFSM();

	/// Handle notifications on parent state changes
	virtual void setupNotifications();
	virtual void setupIopConfiguration();

	/// Action Methods
	virtual void executeWaypointListAction(ExecuteList msg, Receive::Body::ReceiveRec transportData);
	virtual void modifyTravelSpeedAction(ExecuteList msg);
	virtual void resetTravelSpeedAction();
	virtual void sendReportActiveElementAction(QueryActiveElement msg, Receive::Body::ReceiveRec transportData);
	virtual void sendReportLocalWaypointAction(QueryLocalWaypoint msg, Receive::Body::ReceiveRec transportData);
	virtual void sendReportTravelSpeedAction(QueryTravelSpeed msg, Receive::Body::ReceiveRec transportData);


	/// Guard Methods
	virtual bool elementExists(ExecuteList msg);
	virtual bool elementSpecified(ExecuteList msg);
	virtual bool isControllingClient(Receive::Body::ReceiveRec transportData);

	/// ListManagerListenerInterface
	void execute_list(std::vector<iop::InternalElement> elements, double speed = std::numeric_limits<double>::quiet_NaN());
	void stop_execution();

	LocalWaypointListDriver_ReceiveFSMContext *context;

protected:

	/// References to parent FSMs
	urn_jaus_jss_core_ListManager::ListManager_ReceiveFSM* pListManager_ReceiveFSM;
	urn_jaus_jss_core_Management::Management_ReceiveFSM* pManagement_ReceiveFSM;
	urn_jaus_jss_core_AccessControl::AccessControl_ReceiveFSM* pAccessControl_ReceiveFSM;
	urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM;
	urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM;

	std::shared_ptr<iop::Component> cmp;
	rclcpp::Logger logger;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr p_pub_path;
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr p_pub_pose;
	rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr p_pub_tv_max;
	rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr p_sub_finished;
	float p_travel_speed;
	float p_tv_max;
	std::string p_tf_frame_robot;
	std::string p_tf_frame_target;
	ReportLocalWaypoint p_current_waypoint;
	ReportActiveElement p_current_element;
	bool p_executing;
	jUnsignedShortInteger p_last_uid;
	std::unique_ptr<tf2_ros::Buffer> p_tf_buffer;
	std::shared_ptr<tf2_ros::TransformListener> p_tf_listener;

	void pRosFinished(const std_msgs::msg::Bool::SharedPtr state);
	geometry_msgs::msg::PoseStamped get_pose_from_waypoint(iop::InternalElement& element, bool update_current=false);

};

}

#endif // GLOBALWAYPOINTLISTDRIVER_RECEIVEFSM_H
