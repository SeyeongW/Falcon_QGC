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

option(QGC_ENABLE_CESIUM "Enable the CesiumJS mission route view when Qt WebEngine is available" ON)
set(QGC_CESIUM_ION_TOKEN "" CACHE STRING "Cesium ion token embedded in desktop packages")
mark_as_advanced(QGC_CESIUM_ION_TOKEN)

configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/src/CustomCesiumConfig.h.in
    ${CMAKE_CURRENT_BINARY_DIR}/CustomCesiumConfig.h
    @ONLY
)

set(QGC_CESIUM_AVAILABLE OFF)
if(QGC_ENABLE_CESIUM AND NOT ANDROID AND NOT IOS)
    find_package(Qt6 QUIET COMPONENTS WebChannel WebEngineQuick)
    if(TARGET Qt6::WebChannel AND TARGET Qt6::WebEngineQuick)
        set(QGC_CESIUM_AVAILABLE ON)
        message(STATUS "QGC: CesiumJS mission route view enabled")
    else()
        message(WARNING
            "QGC: Qt WebChannel/WebEngineQuick are not available. "
            "The classic mission route view will be used instead."
        )
    endif()
endif()

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

set(CUSTOM_CESIUM_QML_FILES "")
set(CUSTOM_CESIUM_RESOURCES "")
set(CUSTOM_CESIUM_QML_IMPORTS "")
if(QGC_CESIUM_AVAILABLE)
    set(CUSTOM_CESIUM_QML_FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/res/Custom/Widgets/CesiumRoute3DView.qml
    )
    set(CUSTOM_CESIUM_RESOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/res/Custom/Widgets/Cesium/cesium_route.html
        ${CMAKE_CURRENT_SOURCE_DIR}/res/Custom/Widgets/Cesium/cesium_route.js
    )
    set(CUSTOM_CESIUM_QML_IMPORTS QtWebChannel QtWebEngine)
endif()

# Set resource aliases for custom widgets
set_source_files_properties(${CMAKE_CURRENT_SOURCE_DIR}/res/Custom/Widgets/CustomArtificialHorizon.qml PROPERTIES QT_RESOURCE_ALIAS CustomArtificialHorizon.qml)
set_source_files_properties(${CMAKE_CURRENT_SOURCE_DIR}/res/3D/Falcon/generated/Falcon_aircraft.qml PROPERTIES QT_RESOURCE_ALIAS FalconAircraft.qml)
set_source_files_properties(${CMAKE_CURRENT_SOURCE_DIR}/res/3D/Falcon/generated/meshes/aircraft_Body_mesh.mesh PROPERTIES QT_RESOURCE_ALIAS meshes/aircraft_Body_mesh.mesh)
set_source_files_properties(${CMAKE_CURRENT_SOURCE_DIR}/res/3D/Falcon/generated/meshes/epp1045_A_001_mesh.mesh PROPERTIES QT_RESOURCE_ALIAS meshes/epp1045_A_001_mesh.mesh)
set_source_files_properties(${CMAKE_CURRENT_SOURCE_DIR}/res/3D/Falcon/generated/meshes/epp1045_A_mesh.mesh PROPERTIES QT_RESOURCE_ALIAS meshes/epp1045_A_mesh.mesh)
set_source_files_properties(${CMAKE_CURRENT_SOURCE_DIR}/res/3D/Falcon/generated/meshes/epp1045_B_001_mesh.mesh PROPERTIES QT_RESOURCE_ALIAS meshes/epp1045_B_001_mesh.mesh)
set_source_files_properties(${CMAKE_CURRENT_SOURCE_DIR}/res/3D/Falcon/generated/meshes/epp1045_B_002_mesh.mesh PROPERTIES QT_RESOURCE_ALIAS meshes/epp1045_B_002_mesh.mesh)
set_source_files_properties(${CMAKE_CURRENT_SOURCE_DIR}/res/3D/Falcon/generated/meshes/epp1045_B_mesh.mesh PROPERTIES QT_RESOURCE_ALIAS meshes/epp1045_B_mesh.mesh)
if(QGC_CESIUM_AVAILABLE)
    set_source_files_properties(${CMAKE_CURRENT_SOURCE_DIR}/res/Custom/Widgets/Cesium/cesium_route.html PROPERTIES QT_RESOURCE_ALIAS Cesium/cesium_route.html)
    set_source_files_properties(${CMAKE_CURRENT_SOURCE_DIR}/res/Custom/Widgets/Cesium/cesium_route.js PROPERTIES QT_RESOURCE_ALIAS Cesium/cesium_route.js)
endif()
qgc_set_qt_resource_alias(
    ${CMAKE_CURRENT_SOURCE_DIR}/res/Custom/Widgets/ControlSurfaceWidget.qml
    ${CMAKE_CURRENT_SOURCE_DIR}/res/Custom/Widgets/CustomAttitudeWidget.qml
    ${CMAKE_CURRENT_SOURCE_DIR}/res/Custom/Widgets/FalconPfdWidget.qml
    ${CMAKE_CURRENT_SOURCE_DIR}/res/Custom/Widgets/CustomIconButton.qml
    ${CMAKE_CURRENT_SOURCE_DIR}/res/Custom/Widgets/CustomOnOffSwitch.qml
    ${CMAKE_CURRENT_SOURCE_DIR}/res/Custom/Widgets/CustomQuickButton.qml
    ${CMAKE_CURRENT_SOURCE_DIR}/res/Custom/Widgets/CustomSignalStrength.qml
    ${CMAKE_CURRENT_SOURCE_DIR}/res/Custom/Widgets/CustomToolBarButton.qml
    ${CMAKE_CURRENT_SOURCE_DIR}/res/Custom/Widgets/CustomVehicleButton.qml
    ${CMAKE_CURRENT_SOURCE_DIR}/res/Custom/Widgets/MissionRoute3DView.qml
    ${CMAKE_CURRENT_SOURCE_DIR}/res/Custom/Widgets/MissionRouteView.qml
    ${CMAKE_CURRENT_SOURCE_DIR}/res/Custom/Widgets/MovablePanel.qml
    ${CMAKE_CURRENT_SOURCE_DIR}/res/Custom/Widgets/PfdAttitudeView.qml
    ${CMAKE_CURRENT_SOURCE_DIR}/res/Custom/Widgets/PfdHeadingArc.qml
    ${CMAKE_CURRENT_SOURCE_DIR}/res/Custom/Widgets/PfdValueTape.qml
    ${CUSTOM_ROS_QML_FILES}
    ${CUSTOM_CESIUM_QML_FILES}
)

qt_add_qml_module(CustomModule
    URI Custom.Widgets
    VERSION 1.0
    RESOURCE_PREFIX /qml
    IMPORTS
        QtQuick
        QtQuick3D
        ${CUSTOM_CESIUM_QML_IMPORTS}
    QML_FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/res/Custom/Widgets/ControlSurfaceWidget.qml
        ${CMAKE_CURRENT_SOURCE_DIR}/res/Custom/Widgets/CustomArtificialHorizon.qml
        ${CMAKE_CURRENT_SOURCE_DIR}/res/Custom/Widgets/CustomAttitudeWidget.qml
        ${CMAKE_CURRENT_SOURCE_DIR}/res/Custom/Widgets/FalconPfdWidget.qml
        ${CMAKE_CURRENT_SOURCE_DIR}/res/Custom/Widgets/CustomIconButton.qml
        ${CMAKE_CURRENT_SOURCE_DIR}/res/Custom/Widgets/CustomOnOffSwitch.qml
        ${CMAKE_CURRENT_SOURCE_DIR}/res/Custom/Widgets/CustomQuickButton.qml
        ${CMAKE_CURRENT_SOURCE_DIR}/res/Custom/Widgets/CustomSignalStrength.qml
        ${CMAKE_CURRENT_SOURCE_DIR}/res/Custom/Widgets/CustomToolBarButton.qml
        ${CMAKE_CURRENT_SOURCE_DIR}/res/Custom/Widgets/CustomVehicleButton.qml
        ${CMAKE_CURRENT_SOURCE_DIR}/res/3D/Falcon/generated/Falcon_aircraft.qml
        ${CMAKE_CURRENT_SOURCE_DIR}/res/Custom/Widgets/MissionRoute3DView.qml
        ${CMAKE_CURRENT_SOURCE_DIR}/res/Custom/Widgets/MissionRouteView.qml
        ${CMAKE_CURRENT_SOURCE_DIR}/res/Custom/Widgets/MovablePanel.qml
        ${CMAKE_CURRENT_SOURCE_DIR}/res/Custom/Widgets/PfdAttitudeView.qml
        ${CMAKE_CURRENT_SOURCE_DIR}/res/Custom/Widgets/PfdHeadingArc.qml
        ${CMAKE_CURRENT_SOURCE_DIR}/res/Custom/Widgets/PfdValueTape.qml
        ${CUSTOM_ROS_QML_FILES}
        ${CUSTOM_CESIUM_QML_FILES}
    RESOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/res/3D/Falcon/generated/meshes/aircraft_Body_mesh.mesh
        ${CMAKE_CURRENT_SOURCE_DIR}/res/3D/Falcon/generated/meshes/epp1045_A_001_mesh.mesh
        ${CMAKE_CURRENT_SOURCE_DIR}/res/3D/Falcon/generated/meshes/epp1045_A_mesh.mesh
        ${CMAKE_CURRENT_SOURCE_DIR}/res/3D/Falcon/generated/meshes/epp1045_B_001_mesh.mesh
        ${CMAKE_CURRENT_SOURCE_DIR}/res/3D/Falcon/generated/meshes/epp1045_B_002_mesh.mesh
        ${CMAKE_CURRENT_SOURCE_DIR}/res/3D/Falcon/generated/meshes/epp1045_B_mesh.mesh
        ${CUSTOM_CESIUM_RESOURCES}
    NO_PLUGIN
)

target_link_libraries(CustomModule PRIVATE Qt6::Quick3D)
if(QGC_CESIUM_AVAILABLE)
    target_link_libraries(CustomModule PRIVATE Qt6::WebChannel Qt6::WebEngineQuick)
endif()

# The main executable's deployment scan does not inspect imports hidden inside
# this static QML module. Install the two runtime QML plugins explicitly so an
# AppImage can create WebEngineView and QWebChannel without a local Qt SDK.
if(QGC_CESIUM_AVAILABLE AND LINUX AND NOT ANDROID)
    function(qgc_cesium_imported_location imported_target output_variable)
        get_target_property(_imported_configs ${imported_target} IMPORTED_CONFIGURATIONS)
        foreach(_config IN LISTS _imported_configs)
            get_target_property(_imported_path ${imported_target} IMPORTED_LOCATION_${_config})
            if(_imported_path)
                break()
            endif()
        endforeach()

        if(NOT _imported_path)
            message(FATAL_ERROR "QGC: Could not locate imported target ${imported_target}")
        endif()

        set(${output_variable} "${_imported_path}" PARENT_SCOPE)
    endfunction()

    function(qgc_install_cesium_qml_module plugin_target module_name)
        qgc_cesium_imported_location(${plugin_target} _plugin_path)

        cmake_path(GET _plugin_path PARENT_PATH _module_source_dir)
        install(
            DIRECTORY "${_module_source_dir}/"
            DESTINATION "qml/${module_name}"
        )
    endfunction()

    qgc_install_cesium_qml_module(Qt6::WebChannelQuickplugin QtWebChannel)
    qgc_install_cesium_qml_module(Qt6::qtwebenginequickplugin QtWebEngine)

    qgc_cesium_imported_location(
        Qt6::WebEngineQuickDelegatesQml
        _webengine_delegates_library
    )
    cmake_path(GET _webengine_delegates_library PARENT_PATH _qt_library_dir)
    install(
        DIRECTORY "${_qt_library_dir}/"
        DESTINATION lib
        FILES_MATCHING PATTERN "libQt6WebEngineQuickDelegatesQml.so*"
    )
endif()

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
    find_package(fastcdr REQUIRED COMPONENTS shared)
    set_property(TARGET fastcdr PROPERTY IMPORTED_GLOBAL TRUE)
    find_package(rclcpp REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(mavros_msgs REQUIRED)
    find_package(rmw_fastrtps_cpp REQUIRED)

    # ROS middleware and message type-support libraries are loaded with dlopen,
    # so linuxdeploy cannot discover them from the FGC executable dependency
    # graph. Install the runtime entry points explicitly; linuxdeploy will then
    # collect their transitive Fast DDS/message dependencies.
    get_filename_component(QGC_ROS_PREFIX "${rmw_fastrtps_cpp_DIR}/../../.." ABSOLUTE)
    file(GLOB QGC_ROS_TYPESUPPORT_LIBRARIES CONFIGURE_DEPENDS
        "${QGC_ROS_PREFIX}/lib/lib*__rosidl_typesupport_fastrtps_*.so"
        "${QGC_ROS_PREFIX}/lib/lib*__rosidl_typesupport_introspection_*.so"
    )
    if(NOT QGC_ROS_TYPESUPPORT_LIBRARIES)
        message(FATAL_ERROR "QGC: No ROS2 Fast DDS type-support libraries found in ${QGC_ROS_PREFIX}/lib")
    endif()
    set(QGC_ROS_RUNTIME_LIBRARIES
        "${QGC_ROS_PREFIX}/lib/librmw_fastrtps_cpp.so"
        ${QGC_ROS_TYPESUPPORT_LIBRARIES}
    )
    foreach(QGC_ROS_RUNTIME_LIBRARY IN LISTS QGC_ROS_RUNTIME_LIBRARIES)
        if(NOT EXISTS "${QGC_ROS_RUNTIME_LIBRARY}")
            message(FATAL_ERROR "QGC: Required ROS2 runtime library not found: ${QGC_ROS_RUNTIME_LIBRARY}")
        endif()
    endforeach()
    install(FILES ${QGC_ROS_RUNTIME_LIBRARIES} DESTINATION "${CMAKE_INSTALL_LIBDIR}")
    install(
        DIRECTORY "${QGC_ROS_PREFIX}/share/ament_index/"
        DESTINATION "${CMAKE_INSTALL_DATADIR}/ament_index"
    )

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

set(CUSTOM_CESIUM_LIBRARIES "")
set(CUSTOM_CESIUM_DEFINITIONS "")
set(CUSTOM_CESIUM_QT_COMPONENTS "")
if(QGC_CESIUM_AVAILABLE)
    set(CUSTOM_CESIUM_LIBRARIES Qt6::WebChannel Qt6::WebEngineQuick)
    set(CUSTOM_CESIUM_DEFINITIONS QGC_ENABLE_CESIUM)
    set(CUSTOM_CESIUM_QT_COMPONENTS WebChannel WebEngineQuick)
endif()

# Custom libraries to link
set(CUSTOM_LIBRARIES
    CustomModule
    CustomPlanModule
    ${CUSTOM_ROS_LIBRARIES}
    ${CUSTOM_CESIUM_LIBRARIES}
    CACHE INTERNAL "" FORCE
)

# Custom include directories
set(CUSTOM_INCLUDE_DIRECTORIES
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_BINARY_DIR}
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
    ${CUSTOM_CESIUM_DEFINITIONS}
    CACHE INTERNAL "" FORCE
)

# Custom Qt components required
set(CUSTOM_QT_COMPONENTS
    Core
    Positioning
    ${CUSTOM_CESIUM_QT_COMPONENTS}
    CACHE INTERNAL "" FORCE
)
