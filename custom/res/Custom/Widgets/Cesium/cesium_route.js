"use strict";

(function () {
    const CESIUM_URL = "https://cesium.com/downloads/cesiumjs/releases/1.144/Build/Cesium/Cesium.js";
    const LOAD_TIMEOUT_MS = 20000;
    const STATUS_COLORS = ["#64748b", "#d6b84c", "#4e9f75", "#c75c5c"];
    const GROUND_ANCHOR_DEPTH_METERS = 1000;
    const ROUTE_GROUND_CLEARANCE_METERS = 2;
    const OVERVIEW_HEADING_DEGREES = 25;
    const OVERVIEW_PITCH_DEGREES = -35;
    const MINIMUM_FOLLOW_DISTANCE_METERS = 55;
    const MAXIMUM_FOLLOW_DISTANCE_METERS = 120;
    const FOLLOW_ROUTE_RADIUS_SCALE = 0.15;
    const FOLLOW_PITCH_DEGREES = -28;

    let bridge = null;
    let viewer = null;
    let vehicleEntity = null;
    let vehicleParts = [];
    let missionRouteBounds = null;
    let routeAltitudeDatumKey = "";
    let routeAltitudeOffsetMeters = 0;
    let routeBuildRevision = 0;
    let lastRouteSignature = "";
    let lastVehicleSnapshot = null;
    let lastVehiclePosition = null;
    let cameraMode = "auto";
    let cameraFollowing = false;
    let initialViewApplied = false;
    let failed = false;

    function statusElement() {
        return document.getElementById("status");
    }

    function setStatus(message, ready) {
        const element = statusElement();
        element.textContent = message;
        element.classList.toggle("ready", Boolean(ready));
    }

    function reportError(message) {
        if (failed) {
            return;
        }
        failed = true;
        setStatus("3D MAP UNAVAILABLE", false);
        console.error("[FGC Cesium] " + message);
        if (bridge) {
            bridge.reportError(String(message));
        }
    }

    function loadCesium() {
        return new Promise(function (resolve, reject) {
            if (window.Cesium) {
                resolve();
                return;
            }

            const script = document.createElement("script");
            const timeout = window.setTimeout(function () {
                reject(new Error("CesiumJS download timed out"));
            }, LOAD_TIMEOUT_MS);
            script.src = CESIUM_URL;
            script.onload = function () {
                window.clearTimeout(timeout);
                resolve();
            };
            script.onerror = function () {
                window.clearTimeout(timeout);
                reject(new Error("CesiumJS could not be downloaded"));
            };
            document.head.appendChild(script);
        });
    }

    function colorForState(state) {
        const index = Number(state);
        return Cesium.Color.fromCssColorString(STATUS_COLORS[index] || STATUS_COLORS[0]);
    }

    function createViewer() {
        const token = bridge.ionAccessToken || "";
        if (!token) {
            return Promise.reject(new Error("Cesium ion access token is not configured"));
        }
        Cesium.Ion.defaultAccessToken = token;

        return Promise.all([
            Cesium.createWorldImageryAsync(),
            Cesium.createWorldTerrainAsync()
        ]).then(function (providers) {
            const options = {
                animation: false,
                baseLayer: new Cesium.ImageryLayer(providers[0]),
                baseLayerPicker: false,
                fullscreenButton: false,
                geocoder: false,
                homeButton: true,
                infoBox: false,
                navigationHelpButton: false,
                scene3DOnly: true,
                sceneModePicker: false,
                // Render only after a camera/entity/input update. This avoids
                // spending a full GPU frame on an otherwise static map.
                requestRenderMode: true,
                maximumRenderTimeChange: Infinity,
                selectionIndicator: false,
                terrainProvider: providers[1],
                timeline: false
            };

            viewer = new Cesium.Viewer("cesiumContainer", options);
            viewer.scene.globe.depthTestAgainstTerrain = true;
            // Lighting and HDR are expensive on the integrated GPUs commonly
            // used by the mission laptops; imagery/terrain remain unchanged.
            viewer.scene.globe.enableLighting = false;
            viewer.scene.fog.enabled = true;
            viewer.scene.highDynamicRange = false;
            viewer.scene.screenSpaceCameraController.minimumZoomDistance = 5;
            viewer.homeButton.viewModel.command.beforeExecute.addEventListener(
                function (commandInfo) {
                    if (!missionRouteBounds) {
                        return;
                    }
                    commandInfo.cancel = true;
                    setCameraMode("overview");
                }
            );
            initializeCameraControls();

            setStatus("3D TERRAIN READY", false);
            window.setTimeout(function () { setStatus("", true); }, 1800);
        });
    }

    function routePosition(point) {
        return Cesium.Cartesian3.fromDegrees(
            Number(point.longitude),
            Number(point.latitude),
            (Number(point.altitude) || 0) + routeAltitudeOffsetMeters
        );
    }

    function groundAnchorPosition(point) {
        const altitude = (Number(point.altitude) || 0)
            + routeAltitudeOffsetMeters;
        return Cesium.Cartesian3.fromDegrees(
            Number(point.longitude),
            Number(point.latitude),
            Math.min(-GROUND_ANCHOR_DEPTH_METERS,
                     altitude - GROUND_ANCHOR_DEPTH_METERS)
        );
    }

    function focusMissionRoute(duration) {
        if (!viewer || !missionRouteBounds) {
            return;
        }
        viewer.camera.lookAtTransform(Cesium.Matrix4.IDENTITY);
        viewer.camera.flyToBoundingSphere(missionRouteBounds, {
            duration: duration,
            offset: new Cesium.HeadingPitchRange(
                Cesium.Math.toRadians(OVERVIEW_HEADING_DEGREES),
                Cesium.Math.toRadians(OVERVIEW_PITCH_DEGREES),
                Math.max(300, missionRouteBounds.radius * 2.8)
            )
        });
    }

    function updateCameraControls() {
        const controls = document.querySelectorAll("[data-camera-mode]");
        controls.forEach(function (control) {
            control.classList.toggle(
                "selected", control.dataset.cameraMode === cameraMode
            );
        });
    }

    function shouldFollowVehicle(vehicle) {
        if (!vehicle) {
            return false;
        }
        if (cameraMode === "follow") {
            return true;
        }
        return cameraMode === "auto"
            && vehicle.armed
            && (vehicle.flying || vehicle.landing);
    }

    function followVehicle(vehicle, position, preserveZoom) {
        if (!viewer || !vehicle || !position) {
            return;
        }

        const heading = Cesium.Math.toRadians(Number(vehicle.heading) || 0);
        const missionRadius = missionRouteBounds ? missionRouteBounds.radius : 0;
        let followDistance = Math.min(
            MAXIMUM_FOLLOW_DISTANCE_METERS,
            Math.max(MINIMUM_FOLLOW_DISTANCE_METERS,
                     missionRadius * FOLLOW_ROUTE_RADIUS_SCALE)
        );
        if (preserveZoom) {
            const currentDistance = Cesium.Cartesian3.magnitude(
                viewer.camera.position
            );
            if (Number.isFinite(currentDistance)) {
                followDistance = Math.max(
                    viewer.scene.screenSpaceCameraController.minimumZoomDistance,
                    currentDistance
                );
            }
        }

        const lookAheadDistance = Math.min(45, followDistance * 0.18);
        const localLookAhead = new Cesium.Cartesian3(
            Math.sin(heading) * lookAheadDistance,
            Math.cos(heading) * lookAheadDistance,
            2
        );
        const localFrame = Cesium.Transforms.eastNorthUpToFixedFrame(position);
        const worldLookAhead = Cesium.Matrix4.multiplyByPointAsVector(
            localFrame, localLookAhead, new Cesium.Cartesian3()
        );
        const target = Cesium.Cartesian3.add(
            position, worldLookAhead, new Cesium.Cartesian3()
        );

        viewer.camera.lookAt(
            target,
            new Cesium.HeadingPitchRange(
                heading + Math.PI,
                Cesium.Math.toRadians(FOLLOW_PITCH_DEGREES),
                followDistance
            )
        );
    }

    function updateTrackingCamera(vehicle, position) {
        const follow = shouldFollowVehicle(vehicle);
        if (follow) {
            const preserveZoom = cameraFollowing;
            cameraFollowing = true;
            followVehicle(vehicle, position, preserveZoom);
        } else if (cameraFollowing) {
            cameraFollowing = false;
            focusMissionRoute(0.8);
        }
    }

    function setCameraMode(mode) {
        if (mode !== "overview" && mode !== "follow" && mode !== "auto") {
            return;
        }

        cameraMode = mode;
        updateCameraControls();
        if (mode === "overview") {
            cameraFollowing = false;
            focusMissionRoute(0.8);
            return;
        }

        const follow = shouldFollowVehicle(lastVehicleSnapshot);
        const preserveZoom = cameraFollowing && follow;
        cameraFollowing = follow;
        if (follow) {
            followVehicle(lastVehicleSnapshot, lastVehiclePosition, preserveZoom);
        } else {
            focusMissionRoute(0.8);
        }
    }

    function initializeCameraControls() {
        const controls = document.querySelectorAll("[data-camera-mode]");
        controls.forEach(function (control) {
            control.addEventListener("click", function () {
                setCameraMode(control.dataset.cameraMode);
            });
        });
        updateCameraControls();
    }

    function addVehiclePart(id, localOffset, graphicsType, graphics) {
        const entity = viewer.entities.add({
            id: id,
            show: false,
            position: Cesium.Cartesian3.ZERO,
            orientation: Cesium.Quaternion.IDENTITY
        });
        entity[graphicsType] = graphics;
        vehicleParts.push({
            entity: entity,
            localOffset: new Cesium.Cartesian3(
                localOffset[0], localOffset[1], localOffset[2]
            )
        });
        return entity;
    }

    function createVehicleModel() {
        vehicleParts = [];
        const bodyColor = Cesium.Color.fromCssColorString("#d9f3ff");
        const accentColor = Cesium.Color.fromCssColorString("#38bdf8");
        const rotorColor = Cesium.Color.fromCssColorString("#243746");

        vehicleEntity = addVehiclePart(
            "active-vehicle-body",
            [0, 0, 0],
            "ellipsoid",
            new Cesium.EllipsoidGraphics({
                radii: new Cesium.Cartesian3(3.8, 1.05, 0.68),
                material: bodyColor,
                outline: true,
                outlineColor: Cesium.Color.fromCssColorString("#071526")
            })
        );
        vehicleEntity.label = new Cesium.LabelGraphics({
            text: "AIRCRAFT",
            font: "700 11px sans-serif",
            fillColor: bodyColor,
            outlineColor: Cesium.Color.fromCssColorString("#071526"),
            outlineWidth: 4,
            pixelOffset: new Cesium.Cartesian2(0, 24),
            style: Cesium.LabelStyle.FILL_AND_OUTLINE,
            disableDepthTestDistance: 0
        });
        vehicleEntity.point = new Cesium.PointGraphics({
            color: accentColor,
            outlineColor: Cesium.Color.WHITE,
            outlineWidth: 1,
            pixelSize: 9,
            distanceDisplayCondition: new Cesium.DistanceDisplayCondition(150, 150000)
        });

        addVehiclePart(
            "active-vehicle-wing", [0, 0, 0], "box",
            new Cesium.BoxGraphics({
                dimensions: new Cesium.Cartesian3(1.15, 7.2, 0.18),
                material: accentColor
            })
        );
        addVehiclePart(
            "active-vehicle-tail", [-2.8, 0, 0.08], "box",
            new Cesium.BoxGraphics({
                dimensions: new Cesium.Cartesian3(0.8, 3.0, 0.14),
                material: accentColor
            })
        );
        addVehiclePart(
            "active-vehicle-fin", [-3.0, 0, 0.65], "box",
            new Cesium.BoxGraphics({
                dimensions: new Cesium.Cartesian3(0.85, 0.16, 1.35),
                material: bodyColor
            })
        );

        [[1.45, 2.55], [1.45, -2.55], [-1.45, 2.55], [-1.45, -2.55]]
            .forEach(function (offset, index) {
                addVehiclePart(
                    "active-vehicle-rotor-" + index,
                    [offset[0], offset[1], 0.28],
                    "ellipsoid",
                    new Cesium.EllipsoidGraphics({
                        radii: new Cesium.Cartesian3(0.72, 0.72, 0.055),
                        material: rotorColor.withAlpha(0.82)
                    })
                );
            });
    }

    function setVehicleModelVisible(visible) {
        vehicleParts.forEach(function (part) {
            part.entity.show = visible;
        });
    }

    function updateVehicleModelTransform(position, orientation) {
        const rotation = Cesium.Matrix3.fromQuaternion(orientation);
        vehicleParts.forEach(function (part) {
            const worldOffset = Cesium.Matrix3.multiplyByVector(
                rotation, part.localOffset, new Cesium.Cartesian3()
            );
            part.entity.position = Cesium.Cartesian3.add(
                position, worldOffset, new Cesium.Cartesian3()
            );
            part.entity.orientation = orientation;
        });
    }

    function routeAltitudeReference(route) {
        let reference = route[0];
        route.forEach(function (point) {
            if (Number(point.altitude) < Number(reference.altitude)) {
                reference = point;
            }
        });
        return reference;
    }

    function rebuildRoute(route, showDebugSequenceNumbers, currentLegIndex) {
        const revision = ++routeBuildRevision;
        if (!route.length) {
            routeAltitudeDatumKey = "";
            routeAltitudeOffsetMeters = 0;
            buildRoute(route, showDebugSequenceNumbers, currentLegIndex);
            return;
        }

        const reference = routeAltitudeReference(route);
        const referenceAltitude = Number(reference.altitude) || 0;
        const datumKey = Number(reference.latitude).toFixed(7)
            + "," + Number(reference.longitude).toFixed(7)
            + "," + referenceAltitude.toFixed(2);
        if (datumKey === routeAltitudeDatumKey) {
            buildRoute(route, showDebugSequenceNumbers, currentLegIndex);
            return;
        }

        const terrainPosition = Cesium.Cartographic.fromDegrees(
            Number(reference.longitude), Number(reference.latitude)
        );
        Cesium.sampleTerrainMostDetailed(viewer.terrainProvider, [terrainPosition])
            .then(function (positions) {
                if (revision !== routeBuildRevision) {
                    return;
                }
                const terrainHeight = Number(positions[0].height);
                if (!Number.isFinite(terrainHeight)) {
                    throw new Error("Terrain height is unavailable at the mission datum");
                }
                routeAltitudeOffsetMeters = terrainHeight
                    - referenceAltitude
                    + ROUTE_GROUND_CLEARANCE_METERS;
                routeAltitudeDatumKey = datumKey;
                buildRoute(route, showDebugSequenceNumbers, currentLegIndex);
            })
            .catch(function (error) {
                if (revision !== routeBuildRevision) {
                    return;
                }
                console.warn("[FGC Cesium] Route altitude alignment failed:", error);
                routeAltitudeOffsetMeters = 0;
                routeAltitudeDatumKey = datumKey;
                buildRoute(route, showDebugSequenceNumbers, currentLegIndex);
            });
    }

    function buildRoute(route, showDebugSequenceNumbers, currentLegIndex) {
        viewer.entities.removeAll();
        vehicleEntity = null;
        vehicleParts = [];

        if (!route.length) {
            missionRouteBounds = null;
            initialViewApplied = false;
            // Keep a vehicle entity even without a loaded mission. This is the
            // normal manual-flight case and allows live position/attitude
            // updates to remain visible on the Cesium map.
            createVehicleModel();
            return;
        }

        const positions = route.map(routePosition);
        viewer.entities.add({
            id: "mission-route-shadow",
            polyline: {
                positions: positions,
                width: 8,
                material: Cesium.Color.fromCssColorString("#071526").withAlpha(0.76),
                arcType: Cesium.ArcType.GEODESIC
            }
        });
        for (let index = 1; index < route.length; index += 1) {
            const legState = index - 1 === Number(currentLegIndex)
                ? 1 : Number(route[index].state);
            const legColor = legState === 0
                ? Cesium.Color.fromCssColorString("#38bdf8")
                : colorForState(legState);
            viewer.entities.add({
                id: "mission-leg-" + (index - 1),
                polyline: {
                    positions: [positions[index - 1], positions[index]],
                    width: legState === 1 ? 5 : 4,
                    material: legColor,
                    arcType: Cesium.ArcType.GEODESIC
                }
            });
        }

        route.forEach(function (point, index) {
            const debugLabel = showDebugSequenceNumbers
                ? " · SEQ " + point.sequenceNumber : "";
            const waypointColor = colorForState(point.state);
            const waypointPosition = routePosition(point);
            viewer.entities.add({
                id: "mission-ground-anchor-line-" + index,
                polyline: {
                    positions: [groundAnchorPosition(point), waypointPosition],
                    width: 2,
                    material: new Cesium.PolylineDashMaterialProperty({
                        color: waypointColor.withAlpha(0.72),
                        dashLength: 12
                    }),
                    arcType: Cesium.ArcType.NONE
                }
            });
            viewer.entities.add({
                id: "mission-ground-anchor-point-" + index,
                position: Cesium.Cartesian3.fromDegrees(
                    Number(point.longitude), Number(point.latitude)
                ),
                point: {
                    color: waypointColor.withAlpha(0.82),
                    outlineColor: Cesium.Color.WHITE.withAlpha(0.9),
                    outlineWidth: 1,
                    pixelSize: 6,
                    heightReference: Cesium.HeightReference.CLAMP_TO_GROUND
                }
            });
            viewer.entities.add({
                id: "mission-point-" + index,
                position: waypointPosition,
                point: {
                    color: waypointColor,
                    outlineColor: Cesium.Color.WHITE,
                    outlineWidth: 2,
                    pixelSize: point.isTakeoff || point.isLand ? 12 : 9,
                    disableDepthTestDistance: 0
                },
                label: {
                    text: (point.label || "") + debugLabel,
                    font: "600 12px sans-serif",
                    fillColor: Cesium.Color.WHITE,
                    outlineColor: Cesium.Color.fromCssColorString("#071526"),
                    outlineWidth: 4,
                    pixelOffset: new Cesium.Cartesian2(0, -20),
                    style: Cesium.LabelStyle.FILL_AND_OUTLINE,
                    disableDepthTestDistance: 0,
                    distanceDisplayCondition: new Cesium.DistanceDisplayCondition(0, 150000)
                }
            });
        });

        createVehicleModel();

        if (!initialViewApplied) {
            initialViewApplied = true;
            missionRouteBounds = Cesium.BoundingSphere.fromPoints(positions);
            focusMissionRoute(0.8);
        } else {
            missionRouteBounds = Cesium.BoundingSphere.fromPoints(positions);
        }
    }

    function updateVehicle(vehicle) {
        if (!vehicleEntity) {
            return;
        }
        if (!vehicle) {
            lastVehicleSnapshot = null;
            lastVehiclePosition = null;
            setVehicleModelVisible(false);
            updateTrackingCamera(null, null);
            return;
        }

        const position = Cesium.Cartesian3.fromDegrees(
            Number(vehicle.longitude),
            Number(vehicle.latitude),
            (Number(vehicle.altitude) || 0) + routeAltitudeOffsetMeters
        );
        const orientation = Cesium.Transforms.headingPitchRollQuaternion(
            position,
            new Cesium.HeadingPitchRoll(
                Cesium.Math.toRadians((Number(vehicle.heading) || 0) - 90),
                Cesium.Math.toRadians(Number(vehicle.pitch) || 0),
                Cesium.Math.toRadians(Number(vehicle.roll) || 0)
            )
        );
        lastVehicleSnapshot = vehicle;
        lastVehiclePosition = position;
        setVehicleModelVisible(true);
        updateVehicleModelTransform(position, orientation);
        // With no mission bounds there is no overview camera to fit. Establish
        // one initial close view around the live aircraft; subsequent telemetry
        // updates only move the model and do not reset the operator's camera.
        if (!missionRouteBounds && !initialViewApplied) {
            initialViewApplied = true;
            followVehicle(vehicle, position, false);
        }
        updateTrackingCamera(vehicle, position);
    }

    function renderSnapshot(json) {
        if (!viewer || !json) {
            return;
        }

        let snapshot;
        try {
            snapshot = JSON.parse(json);
        } catch (error) {
            reportError("Invalid route data: " + error.message);
            return;
        }

        const route = Array.isArray(snapshot.route) ? snapshot.route : [];
        // Vehicle telemetry and waypoint progress arrive much more frequently
        // than mission geometry. Rebuilding all entities for a state/index-only
        // change clears the scene for a frame and appears as flicker, so use
        // only geometry and labels for the rebuild signature.
        const signature = JSON.stringify({
            route: route.map(function (point) {
                return {
                    latitude: Number(point.latitude),
                    longitude: Number(point.longitude),
                    altitude: Number(point.altitude),
                    label: point.label || "",
                    isTakeoff: Boolean(point.isTakeoff),
                    isLand: Boolean(point.isLand),
                    isWaypoint: Boolean(point.isWaypoint)
                };
            }),
            debug: Boolean(snapshot.showDebugSequenceNumbers)
        });
        if (signature !== lastRouteSignature) {
            lastRouteSignature = signature;
            rebuildRoute(route,
                         Boolean(snapshot.showDebugSequenceNumbers),
                         Number(snapshot.currentLegIndex));
        }
        updateVehicle(snapshot.vehicle || null);
        viewer.scene.requestRender();
    }

    window.addEventListener("error", function (event) {
        reportError(event.message || "Unexpected CesiumJS error");
    });
    window.addEventListener("unhandledrejection", function (event) {
        const reason = event.reason && event.reason.message
            ? event.reason.message : String(event.reason);
        reportError(reason);
    });

    if (!window.qt || !qt.webChannelTransport) {
        reportError("Qt WebChannel transport is unavailable");
        return;
    }

    new QWebChannel(qt.webChannelTransport, function (channel) {
        bridge = channel.objects.fgcBridge;
        bridge.snapshotJsonChanged.connect(function () {
            renderSnapshot(bridge.snapshotJson);
        });

        loadCesium()
            .then(function () {
                return createViewer();
            })
            .then(function () {
                renderSnapshot(bridge.snapshotJson);
                bridge.reportReady();
            })
            .catch(function (error) {
                reportError(error.message || String(error));
            });
    });
}());
