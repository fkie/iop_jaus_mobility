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

#include <pluginlib/class_list_macros.h>
#include "LocalWaypointListDriverPlugin.h"

using namespace iop;
using namespace urn_jaus_jss_mobility_LocalWaypointListDriver;
using namespace urn_jaus_jss_core_ListManager;
using namespace urn_jaus_jss_core_Management;
using namespace urn_jaus_jss_core_AccessControl;
using namespace urn_jaus_jss_core_Events;
using namespace urn_jaus_jss_core_Transport;


LocalWaypointListDriverPlugin::LocalWaypointListDriverPlugin()
{
	p_my_service = NULL;
	p_base_service = NULL;
	p_mgmt_service = NULL;
	p_accesscontrol_service = NULL;
	p_events_service = NULL;
	p_transport_service = NULL;
}

JTS::Service* LocalWaypointListDriverPlugin::get_service()
{
	return p_my_service;
}

void LocalWaypointListDriverPlugin::create_service(JTS::JausRouter* jaus_router)
{
	p_base_service = static_cast<ListManagerService *>(get_base_service());
	p_mgmt_service = static_cast<ManagementService *>(get_base_service(2));
	p_accesscontrol_service = static_cast<AccessControlService *>(get_base_service(3));
	p_events_service = static_cast<EventsService *>(get_base_service(4));
	p_transport_service = static_cast<TransportService *>(get_base_service(5));
	p_my_service = new LocalWaypointListDriverService(jaus_router, p_transport_service, p_events_service, p_accesscontrol_service, p_mgmt_service, p_base_service);
}

PLUGINLIB_EXPORT_CLASS(iop::LocalWaypointListDriverPlugin, iop::PluginInterface)
