

#ifndef GLOBALWAYPOINTDRIVER_RECEIVEFSM_H
#define GLOBALWAYPOINTDRIVER_RECEIVEFSM_H

#include "JausUtils.h"
#include "InternalEvents/InternalEventHandler.h"
#include "Transport/JausTransport.h"
#include "JTSStateMachine.h"
#include "urn_jaus_jss_mobility_GlobalWaypointDriver/Messages/MessageSet.h"
#include "urn_jaus_jss_mobility_GlobalWaypointDriver/InternalEvents/InternalEventsSet.h"

#include "InternalEvents/Receive.h"
#include "InternalEvents/Send.h"

#include "urn_jaus_jss_core_Transport/Transport_ReceiveFSM.h"
#include "urn_jaus_jss_core_Events/Events_ReceiveFSM.h"
#include "urn_jaus_jss_core_AccessControl/AccessControl_ReceiveFSM.h"
#include "urn_jaus_jss_core_Management/Management_ReceiveFSM.h"

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>


#include "GlobalWaypointDriver_ReceiveFSM_sm.h"

namespace urn_jaus_jss_mobility_GlobalWaypointDriver
{

class DllExport GlobalWaypointDriver_ReceiveFSM : public JTS::StateMachine
{
public:
	GlobalWaypointDriver_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM, urn_jaus_jss_core_AccessControl::AccessControl_ReceiveFSM* pAccessControl_ReceiveFSM, urn_jaus_jss_core_Management::Management_ReceiveFSM* pManagement_ReceiveFSM);
	virtual ~GlobalWaypointDriver_ReceiveFSM();

	/// Handle notifications on parent state changes
	virtual void setupNotifications();

	/// Action Methods
	virtual void resetTravelSpeedAction();
	virtual void sendReportGlobalWaypointAction(QueryGlobalWaypoint msg, Receive::Body::ReceiveRec transportData);
	virtual void sendReportTravelSpeedAction(QueryTravelSpeed msg, Receive::Body::ReceiveRec transportData);
	virtual void setTravelSpeedAction(SetTravelSpeed msg, Receive::Body::ReceiveRec transportData);
	virtual void setWaypointAction(SetGlobalWaypoint msg, Receive::Body::ReceiveRec transportData);


	/// Guard Methods
	virtual bool isControllingClient(Receive::Body::ReceiveRec transportData);
	virtual bool waypointExists(QueryGlobalWaypoint msg);
	virtual bool waypointExists(SetTravelSpeed msg);


	GlobalWaypointDriver_ReceiveFSMContext *context;

protected:

    /// References to parent FSMs
	urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM;
	urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM;
	urn_jaus_jss_core_AccessControl::AccessControl_ReceiveFSM* pAccessControl_ReceiveFSM;
	urn_jaus_jss_core_Management::Management_ReceiveFSM* pManagement_ReceiveFSM;

	ros::Publisher p_pub_path;
	ros::Publisher p_pub_pose;
	ros::Publisher p_pub_fix;
	ros::Publisher p_pub_tv_max;
	ros::Subscriber p_sub_finished;
	float p_travel_speed;
	float p_tv_max;
	std::string p_tf_frame_world;
	ReportGlobalWaypoint p_current_waypoint;

	void pStop();
	void pRosFinished(const std_msgs::Bool::ConstPtr& state);

};

};

#endif // GLOBALWAYPOINTDRIVER_RECEIVEFSM_H
