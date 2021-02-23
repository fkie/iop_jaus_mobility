

#ifndef LOCALWAYPOINTDRIVER_RECEIVEFSM_H
#define LOCALWAYPOINTDRIVER_RECEIVEFSM_H

#include "JausUtils.h"
#include "InternalEvents/InternalEventHandler.h"
#include "Transport/JausTransport.h"
#include "JTSStateMachine.h"
#include "urn_jaus_jss_mobility_LocalWaypointDriver/Messages/MessageSet.h"
#include "urn_jaus_jss_mobility_LocalWaypointDriver/InternalEvents/InternalEventsSet.h"

#include "InternalEvents/Receive.h"
#include "InternalEvents/Send.h"

#include "urn_jaus_jss_core_Management/Management_ReceiveFSM.h"
#include "urn_jaus_jss_core_AccessControl/AccessControl_ReceiveFSM.h"
#include "urn_jaus_jss_core_Events/Events_ReceiveFSM.h"
#include "urn_jaus_jss_core_Transport/Transport_ReceiveFSM.h"


#include "LocalWaypointDriver_ReceiveFSM_sm.h"
#include <rclcpp/rclcpp.hpp>
#include <fkie_iop_component/iop_component.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>

namespace urn_jaus_jss_mobility_LocalWaypointDriver
{

class DllExport LocalWaypointDriver_ReceiveFSM : public JTS::StateMachine
{
public:
	LocalWaypointDriver_ReceiveFSM(std::shared_ptr<iop::Component> cmp, urn_jaus_jss_core_Management::Management_ReceiveFSM* pManagement_ReceiveFSM, urn_jaus_jss_core_AccessControl::AccessControl_ReceiveFSM* pAccessControl_ReceiveFSM, urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM, urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM);
	virtual ~LocalWaypointDriver_ReceiveFSM();

	/// Handle notifications on parent state changes
	virtual void setupNotifications();
	virtual void setupIopConfiguration();

	/// Action Methods
	virtual void resetTravelSpeedAction();
	virtual void sendReportLocalWaypointAction(QueryLocalWaypoint msg, Receive::Body::ReceiveRec transportData);
	virtual void sendReportTravelSpeedAction(QueryTravelSpeed msg, Receive::Body::ReceiveRec transportData);
	virtual void setTravelSpeedAction(SetTravelSpeed msg, Receive::Body::ReceiveRec transportData);
	virtual void setWaypointAction(SetLocalWaypoint msg, Receive::Body::ReceiveRec transportData);


	/// Guard Methods
	virtual bool isControllingClient(Receive::Body::ReceiveRec transportData);
	virtual bool waypointExists(QueryLocalWaypoint msg);
	virtual bool waypointExists(SetTravelSpeed msg);


	LocalWaypointDriver_ReceiveFSMContext *context;

protected:

	/// References to parent FSMs
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
	ReportLocalWaypoint p_current_waypoint;

	void pStop();
	void pRosFinished(const std_msgs::msg::Bool::SharedPtr state);

};

}

#endif // LOCALWAYPOINTDRIVER_RECEIVEFSM_H
