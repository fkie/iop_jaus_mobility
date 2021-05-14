

#ifndef GLOBALWAYPOINTLISTDRIVER_RECEIVEFSM_H
#define GLOBALWAYPOINTLISTDRIVER_RECEIVEFSM_H

#include "JausUtils.h"
#include "InternalEvents/InternalEventHandler.h"
#include "Transport/JausTransport.h"
#include "JTSStateMachine.h"
#include "urn_jaus_jss_mobility_GlobalWaypointListDriver/Messages/MessageSet.h"
#include "urn_jaus_jss_mobility_GlobalWaypointListDriver/InternalEvents/InternalEventsSet.h"

#include "InternalEvents/Receive.h"
#include "InternalEvents/Send.h"

#include "urn_jaus_jss_core_Transport/Transport_ReceiveFSM.h"
#include "urn_jaus_jss_core_Events/Events_ReceiveFSM.h"
#include "urn_jaus_jss_core_AccessControl/AccessControl_ReceiveFSM.h"
#include "urn_jaus_jss_core_Management/Management_ReceiveFSM.h"
#include "urn_jaus_jss_core_ListManager/ListManager_ReceiveFSM.h"

#include <ros/ros.h>
#include <geographic_msgs/GeoPath.h>
#include <geographic_msgs/GeoPoseStamped.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

#include <fkie_iop_list_manager/ListManagerListenerInterface.h>

#include "GlobalWaypointListDriver_ReceiveFSM_sm.h"

namespace urn_jaus_jss_mobility_GlobalWaypointListDriver
{

class DllExport GlobalWaypointListDriver_ReceiveFSM : public JTS::StateMachine, public iop::ListManagerListenerInterface
{
public:
	GlobalWaypointListDriver_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM, urn_jaus_jss_core_AccessControl::AccessControl_ReceiveFSM* pAccessControl_ReceiveFSM, urn_jaus_jss_core_Management::Management_ReceiveFSM* pManagement_ReceiveFSM, urn_jaus_jss_core_ListManager::ListManager_ReceiveFSM* pListManager_ReceiveFSM);
	virtual ~GlobalWaypointListDriver_ReceiveFSM();

	/// Handle notifications on parent state changes
	virtual void setupNotifications();

	/// Action Methods
	virtual void executeWaypointListAction(ExecuteList msg, Receive::Body::ReceiveRec transportData);
	virtual void modifyTravelSpeedAction(ExecuteList msg);
	virtual void resetTravelSpeedAction();
	virtual void sendReportActiveElementAction(QueryActiveElement msg, Receive::Body::ReceiveRec transportData);
	virtual void sendReportGlobalWaypointAction(QueryGlobalWaypoint msg, Receive::Body::ReceiveRec transportData);
	virtual void sendReportTravelSpeedAction(QueryTravelSpeed msg, Receive::Body::ReceiveRec transportData);


	/// Guard Methods
	virtual bool elementExists(ExecuteList msg);
	virtual bool elementSpecified(ExecuteList msg);
	virtual bool isControllingClient(Receive::Body::ReceiveRec transportData);

	/// ListManagerListenerInterface
	void execute_list(std::vector<iop::InternalElement> elements, double speed = std::numeric_limits<double>::quiet_NaN());
	void stop_execution();

	GlobalWaypointListDriver_ReceiveFSMContext *context;

protected:

    /// References to parent FSMs
	urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM;
	urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM;
	urn_jaus_jss_core_AccessControl::AccessControl_ReceiveFSM* pAccessControl_ReceiveFSM;
	urn_jaus_jss_core_Management::Management_ReceiveFSM* pManagement_ReceiveFSM;
	urn_jaus_jss_core_ListManager::ListManager_ReceiveFSM* pListManager_ReceiveFSM;

	ros::Publisher p_pub_path;
	ros::Publisher p_pub_tv_max;
	ros::Subscriber p_sub_finished;
	ros::Publisher p_pub_geopath;
	float p_travel_speed;
	float p_tv_max;
	std::string p_tf_frame_world;
	ReportGlobalWaypoint p_current_waypoint;
	ReportActiveElement p_current_element;
	bool p_executing;
	jUnsignedShortInteger p_last_uid;

	void pRosFinished(const std_msgs::Bool::ConstPtr& state);
	geometry_msgs::PoseStamped get_pose_from_waypoint(iop::InternalElement& element, bool update_current=false);
	geographic_msgs::GeoPoseStamped get_geopose_from_waypoint(iop::InternalElement& element, bool update_current=false);

};

};

#endif // GLOBALWAYPOINTLISTDRIVER_RECEIVEFSM_H
