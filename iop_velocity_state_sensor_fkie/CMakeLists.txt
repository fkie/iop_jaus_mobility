cmake_minimum_required(VERSION 2.8.3)
project(iop_velocity_state_sensor_fkie)

find_package(catkin REQUIRED COMPONENTS
			 iop_component_fkie
             iop_management_fkie
             nav_msgs
             roscpp
) 

iop_init(COMPONENT_ID 0)

catkin_package(
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
        iop_component_fkie
        iop_management_fkie
        nav_msgs
)

iop_code_generator(
  IDLS
    urn.jaus.jss.core-v1.0/Events.xml
    urn.jaus.jss.core-v1.0/Transport.xml
    urn.jaus.jss.mobility/VelocityStateSensor.xml
  OWN_IDLS
  OVERRIDES
    include/urn_jaus_jss_mobility_VelocityStateSensor/VelocityStateSensor_ReceiveFSM.h
    src/urn_jaus_jss_mobility_VelocityStateSensor/VelocityStateSensor_ReceiveFSM.cpp
    src/main.cpp
  EXTERN_SERVICES
    urn_jaus_jss_core_Events
    urn_jaus_jss_core_Transport
  GENERATED_SOURCES cppfiles
)

include_directories(${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
            src/VelocityStateSensorPlugin.cpp
            ${cppfiles}
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
                      ${catkin_LIBRARIES}
)

install(
  DIRECTORY ${IOP_INSTALL_INCLUDE_DIRS} DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  PATTERN "*.old" EXCLUDE
  PATTERN "*.gen" EXCLUDE
)

# Mark executables and/or libraries for installation
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