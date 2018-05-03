cmake_minimum_required(VERSION 2.8.3)
project(iop_local_waypoint_driver_fkie)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
             geometry_msgs
             iop_component_fkie
             iop_management_fkie
             nav_msgs
             roscpp
             std_msgs
             tf
)
iop_init(COMPONENT_ID 0)
iop_export_service(urn_jaus_jss_mobility_LocalWaypointDriver)

###################################
## catkin specific configuration ##
###################################
catkin_package(
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
        geometry_msgs
        iop_component_fkie
        iop_management_fkie
        nav_msgs
        std_msgs
        tf
)

###########
## Build ##
###########

## Specify additional locations of header files
iop_code_generator(
  IDLS
    urn.jaus.jss.core-v1.0/AccessControl.xml
    urn.jaus.jss.core-v1.0/Events.xml
    urn.jaus.jss.core-v1.0/Management.xml
    urn.jaus.jss.core-v1.0/Transport.xml
    urn.jaus.jss.mobility/LocalWaypointDriver.xml
  OWN_IDLS
  OVERRIDES
    include/urn_jaus_jss_mobility_LocalWaypointDriver/LocalWaypointDriver_ReceiveFSM.h
    src/urn_jaus_jss_mobility_LocalWaypointDriver/LocalWaypointDriver_ReceiveFSM.cpp
    src/main.cpp
  EXTERN_SERVICES
    urn_jaus_jss_core_AccessControl
    urn_jaus_jss_core_Events
    urn_jaus_jss_core_Management
    urn_jaus_jss_core_Transport
  GENERATED_SOURCES cppfiles
)

include_directories(${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
            src/LocalWaypointDriverPlugin.cpp
            ${cppfiles}
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
                      ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(
  DIRECTORY ${IOP_INSTALL_INCLUDE_DIRS} DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  PATTERN "*.old" EXCLUDE
  PATTERN "*.gen" EXCLUDE
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(
   FILES ./plugin_iop.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

