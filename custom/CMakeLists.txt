# ============================================================================
# QGroundControl Custom Plugin Example
# Template for building custom QGroundControl plugins
# ============================================================================

message(STATUS "QGC: Adding Custom Plugin")

# ============================================================================
# Custom Android Package Template
# ============================================================================

if(ANDROID)
    set(CUSTOM_ANDROID_DIR "${CMAKE_SOURCE_DIR}/custom/android")
    if(EXISTS "${CUSTOM_ANDROID_DIR}")
        file(GLOB CUSTOM_ANDROID_FILES "${CUSTOM_ANDROID_DIR}/*")
        if(CUSTOM_ANDROID_FILES)
            message(STATUS "QGC: Custom Android package template found. Overlaying custom files...")
            set(DEFAULT_ANDROID_DIR "${CMAKE_SOURCE_DIR}/android")
            set(FINAL_ANDROID_DIR "${CMAKE_BINARY_DIR}/custom/android")

            # Copy default Android template
            file(COPY "${DEFAULT_ANDROID_DIR}/." DESTINATION "${FINAL_ANDROID_DIR}")

            # Overlay custom Android files
            execute_process(COMMAND ${CMAKE_COMMAND} -E copy_directory "${CUSTOM_ANDROID_DIR}" "${FINAL_ANDROID_DIR}")

            # Track changes to Android directories
            set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
                "${DEFAULT_ANDROID_DIR}/"
                "${CUSTOM_ANDROID_DIR}/"
            )

            set(QGC_ANDROID_PACKAGE_SOURCE_DIR "${FINAL_ANDROID_DIR}" CACHE PATH "Path to a custom Android package template" FORCE)
            message(STATUS "QGC: Android package template path will be set to: ${QGC_ANDROID_PACKAGE_SOURCE_DIR}")
        else()
            message(STATUS "QGC: Custom Android package template empty. Using default.")
        endif()
    else()
        message(STATUS "QGC: No custom Android package template found. Using default.")
    endif()
endif()

# ============================================================================
# Custom Resources and Import Paths
# ============================================================================

# Add custom Qt resources
list(APPEND CUSTOM_RESOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/custom.qrc
)
set(QGC_RESOURCES ${QGC_RESOURCES} ${CUSTOM_RESOURCES} CACHE STRING "Paths to .qrc Resources" FORCE)

# Add custom QML import path
set(QML_IMPORT_PATH ${QML_IMPORT_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/res" CACHE STRING "Extra qml import paths" FORCE)

# ============================================================================
# Custom QML Module
# ============================================================================

qt_add_library(CustomModule STATIC)

# The ROS video panel imports Custom.Ros, so it is only part of the module when
# the ROS bridge is enabled (otherwise it would fail to load in a non-ROS build).
set(CUSTOM_ROS_QML_FILES "")
if(QGC_ENABLE_ROS)
    set(CUSTOM_ROS_QML_FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/res/Custom/Widgets/RosVideoPanel.qml
        ${CMAKE_CURRENT_SOURCE_DIR}/res/Custom/Widgets/RosActuatorSource.qml
        ${CMAKE_CURRENT_SOURCE_DIR}/res/Custom/Widgets/MissionPhasePanel.qml
    )
endif()

# Set resource aliases for custom widgets
set_source_files_properties(${CMAKE_CURRENT_SOURCE_DIR}/res/Custom/Widgets/CustomArtificialHorizon.qml PROPERTIES QT_RESOURCE_ALIAS CustomArtificialHorizon.qml)
qgc_set_qt_resource_alias(
    ${CMAKE_CURRENT_SOURCE_DIR}/res/Custom/Widgets/ControlSurfaceWidget.qml
    ${CMAKE_CURRENT_SOURCE_DIR}/res/Custom/Widgets/CustomAttitudeWidget.qml
    ${CMAKE_CURRENT_SOURCE_DIR}/res/Custom/Widgets/CustomIconButton.qml
    ${CMAKE_CURRENT_SOURCE_DIR}/res/Custom/Widgets/CustomOnOffSwitch.qml
    ${CMAKE_CURRENT_SOURCE_DIR}/res/Custom/Widgets/CustomQuickButton.qml
    ${CMAKE_CURRENT_SOURCE_DIR}/res/Custom/Widgets/CustomSignalStrength.qml
    ${CMAKE_CURRENT_SOURCE_DIR}/res/Custom/Widgets/CustomToolBarButton.qml
    ${CMAKE_CURRENT_SOURCE_DIR}/res/Custom/Widgets/CustomVehicleButton.qml
    ${CUSTOM_ROS_QML_FILES}
)

qt_add_qml_module(CustomModule
    URI Custom.Widgets
    VERSION 1.0
    RESOURCE_PREFIX /qml
    QML_FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/res/Custom/Widgets/ControlSurfaceWidget.qml
        ${CMAKE_CURRENT_SOURCE_DIR}/res/Custom/Widgets/CustomArtificialHorizon.qml
        ${CMAKE_CURRENT_SOURCE_DIR}/res/Custom/Widgets/CustomAttitudeWidget.qml
        ${CMAKE_CURRENT_SOURCE_DIR}/res/Custom/Widgets/CustomIconButton.qml
        ${CMAKE_CURRENT_SOURCE_DIR}/res/Custom/Widgets/CustomOnOffSwitch.qml
        ${CMAKE_CURRENT_SOURCE_DIR}/res/Custom/Widgets/CustomQuickButton.qml
        ${CMAKE_CURRENT_SOURCE_DIR}/res/Custom/Widgets/CustomSignalStrength.qml
        ${CMAKE_CURRENT_SOURCE_DIR}/res/Custom/Widgets/CustomToolBarButton.qml
        ${CMAKE_CURRENT_SOURCE_DIR}/res/Custom/Widgets/CustomVehicleButton.qml
        ${CUSTOM_ROS_QML_FILES}
    NO_PLUGIN
)

# ============================================================================
# Custom Plan Module (PerimeterScan map visual + editor)
# ============================================================================

qt_add_library(CustomPlanModule STATIC)

qgc_set_qt_resource_alias(
    ${CMAKE_CURRENT_SOURCE_DIR}/src/PlanView/PerimeterScanMapVisual.qml
    ${CMAKE_CURRENT_SOURCE_DIR}/src/PlanView/PerimeterScanEditor.qml
)

qt_add_qml_module(CustomPlanModule
    URI Custom.Plan
    VERSION 1.0
    RESOURCE_PREFIX /qml
    QML_FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/src/PlanView/PerimeterScanMapVisual.qml
        ${CMAKE_CURRENT_SOURCE_DIR}/src/PlanView/PerimeterScanEditor.qml
    NO_PLUGIN
)

# ============================================================================
# Optional ROS2 Bridge (VTOL-GCS)
# ============================================================================
# Links rclcpp into QGC so recognition video / mission-state / gripper topics
# can be consumed directly. Linux-only and OFF by default: enabling it makes
# the build depend on a sourced ROS2 environment (find_package(rclcpp)) and
# breaks Windows/Android builds. Configure with `-DQGC_ENABLE_ROS=ON` from a
# shell that has `source /opt/ros/<distro>/setup.bash`.

option(QGC_ENABLE_ROS "Build the VTOL-GCS ROS2 bridge (requires a sourced ROS2 env)" OFF)

if(QGC_ENABLE_ROS)
    message(STATUS "QGC: ROS2 bridge enabled")
    find_package(Qt6 REQUIRED COMPONENTS Core Gui Quick)
    find_package(rclcpp REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(mavros_msgs REQUIRED)

    add_library(CustomRosBridge STATIC
        ${CMAKE_CURRENT_SOURCE_DIR}/src/Ros/RosBridge.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/src/Ros/RosBridge.h
        ${CMAKE_CURRENT_SOURCE_DIR}/src/Ros/RosVideoView.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/src/Ros/RosVideoView.h
    )
    set_target_properties(CustomRosBridge PROPERTIES
        AUTOMOC ON
        POSITION_INDEPENDENT_CODE ON
    )
    target_include_directories(CustomRosBridge PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/src/Ros
    )
    target_link_libraries(CustomRosBridge PUBLIC Qt6::Core Qt6::Gui Qt6::Quick)
    ament_target_dependencies(CustomRosBridge PUBLIC rclcpp std_msgs sensor_msgs mavros_msgs)
endif()

# ============================================================================
# Custom Plugin Sources
# ============================================================================

set(CUSTOM_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/CustomPlugin.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/CustomPlugin.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/AutoPilotPlugin/CustomAutoPilotPlugin.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/AutoPilotPlugin/CustomAutoPilotPlugin.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/FirmwarePlugin/CustomFirmwarePlugin.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/FirmwarePlugin/CustomFirmwarePlugin.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/FirmwarePlugin/CustomFirmwarePluginFactory.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/FirmwarePlugin/CustomFirmwarePluginFactory.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/MissionManager/PerimeterScanComplexItem.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/MissionManager/PerimeterScanComplexItem.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/MissionManager/PerimeterScanPlanCreator.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/MissionManager/PerimeterScanPlanCreator.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Position/CustomGeoclueSource.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Position/CustomGeoclueSource.h
    CACHE INTERNAL "" FORCE
)

# ============================================================================
# Custom Build Configuration
# ============================================================================

# Optional ROS bridge additions (empty unless QGC_ENABLE_ROS)
set(CUSTOM_ROS_LIBRARIES "")
set(CUSTOM_ROS_INCLUDE_DIRECTORIES "")
set(CUSTOM_ROS_DEFINITIONS "")
if(QGC_ENABLE_ROS)
    set(CUSTOM_ROS_LIBRARIES CustomRosBridge)
    set(CUSTOM_ROS_INCLUDE_DIRECTORIES ${CMAKE_CURRENT_SOURCE_DIR}/src/Ros)
    set(CUSTOM_ROS_DEFINITIONS QGC_ENABLE_ROS)
endif()

# Custom libraries to link
set(CUSTOM_LIBRARIES
    CustomModule
    CustomPlanModule
    ${CUSTOM_ROS_LIBRARIES}
    CACHE INTERNAL "" FORCE
)

# Custom include directories
set(CUSTOM_INCLUDE_DIRECTORIES
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/AutoPilotPlugin
    ${CMAKE_CURRENT_SOURCE_DIR}/src/FirmwarePlugin
    ${CMAKE_CURRENT_SOURCE_DIR}/src/MissionManager
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Position
    ${CUSTOM_ROS_INCLUDE_DIRECTORIES}
    CACHE INTERNAL "" FORCE
)

# Custom compiler definitions
set(CUSTOM_DEFINITIONS
    QGC_CUSTOM_BUILD
    CUSTOMHEADER="CustomPlugin.h"
    CUSTOMCLASS=CustomPlugin
    ${CUSTOM_ROS_DEFINITIONS}
    CACHE INTERNAL "" FORCE
)

# Custom Qt components required
set(CUSTOM_QT_COMPONENTS
    Core
    Positioning
    CACHE INTERNAL "" FORCE
)
