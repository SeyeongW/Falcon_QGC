import QtQuick
import QtQuick3D

Node {
    id: node

    readonly property alias aircraftBodyNode: aircraft_Body
    readonly property alias liftPropFlNode: lift_prop_fl
    readonly property alias liftPropFrNode: lift_prop_fr
    readonly property alias liftPropRlNode: lift_prop_rl
    readonly property alias liftPropRrNode: lift_prop_rr
    readonly property alias pusherPropNode: pusher_prop

    // Resources
    PrincipledMaterial {
        id: material_001_material
        objectName: "Material.001"
        baseColor: "#ffe7e7e7"
        roughness: 0.5
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: material_002_material
        objectName: "Material.002"
        baseColor: "#ffe7e7e7"
        roughness: 0.5
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: material_003_material
        objectName: "Material.003"
        baseColor: "#ffe7e7e7"
        roughness: 0.5
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: material_004_material
        objectName: "Material.004"
        baseColor: "#ffe7e7e7"
        roughness: 0.5
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: principledMaterial
        metalness: 1
        roughness: 1
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: material_material
        objectName: "Material"
        baseColor: "#ffe7e7e7"
        roughness: 0.5
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }

    // Nodes:
    Node {
        id: root
        objectName: "ROOT"
        Model {
            id: aircraft_Body
            objectName: "Aircraft_Body"
            position: Qt.vector3d(-0.207799, 0.109556, -1.49151)
            rotation: Qt.quaternion(0.0534344, 0.181108, -0.686577, 0.702108)
            scale: Qt.vector3d(0.0014, 0.0014, 0.0014)
            source: "meshes/aircraft_Body_mesh.mesh"
            materials: [
                material_001_material,
                material_002_material,
                material_003_material,
                material_004_material,
                principledMaterial,
                principledMaterial,
                principledMaterial,
                principledMaterial
            ]
        }
        Model {
            id: lift_prop_rr
            objectName: "lift_prop_rr"
            position: Qt.vector3d(-0.525326, 0.143201, 0.669437)
            source: "meshes/epp1045_B_mesh.mesh"
            materials: [
                material_material
            ]
        }
        Model {
            id: lift_prop_fl
            objectName: "lift_prop_fl"
            position: Qt.vector3d(0.492242, 0.1432, -0.669747)
            source: "meshes/epp1045_B_001_mesh.mesh"
            materials: [
                material_material
            ]
        }
        Model {
            id: lift_prop_fr
            objectName: "lift_prop_fr"
            position: Qt.vector3d(0.491545, 0.143196, 0.669954)
            source: "meshes/epp1045_A_mesh.mesh"
            materials: [
                material_material
            ]
        }
        Model {
            id: lift_prop_rl
            objectName: "lift_prop_rl"
            position: Qt.vector3d(-0.528989, 0.143196, -0.66923)
            source: "meshes/epp1045_A_001_mesh.mesh"
            materials: [
                material_material
            ]
        }
        Model {
            id: pusher_prop
            objectName: "pusher_prop"
            position: Qt.vector3d(-1.23383, -0.000398465, -4.42225e-05)
            rotation: Qt.quaternion(0.707107, 0, 0, 0.707107)
            scale: Qt.vector3d(1, 1, 1)
            source: "meshes/epp1045_B_002_mesh.mesh"
            materials: [
                material_material
            ]
        }
    }

    // Animations:
}
