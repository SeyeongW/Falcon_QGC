pragma ComponentBehavior: Bound

import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import QGroundControl
import QGroundControl.Controls
import QGroundControl.AnalyzeView

AnalyzePage {
    id: root

    pageComponent: pageComponent
    pageDescription: qsTr("Convert a PX4 ULog into the 10 Hz LLA ASCII format required for competition flight-data submission.")
    allowPopout: true

    Component {
        id: pageComponent

        ColumnLayout {
            width: availableWidth
            height: availableHeight
            spacing: ScreenTools.defaultFontPixelHeight * 0.6

            CompetitionDataController {
                id: controller
            }

            Component.onDestruction: controller.clear()

            RowLayout {
                Layout.fillWidth: true
                spacing: ScreenTools.defaultFontPixelWidth

                QGCButton {
                    text: qsTr("Open PX4 ULog")
                    enabled: !controller.parsing
                    onClicked: openDialog.openForLoad()
                }

                QGCButton {
                    text: qsTr("Export 10 Hz ASCII")
                    enabled: controller.ready && !controller.parsing
                    onClicked: {
                        saveDialog.folder = controller.suggestedOutputPath.replace(/[/\\][^/\\]*$/, "")
                        saveDialog.openForSave()
                    }
                }

                QGCButton {
                    text: qsTr("Clear")
                    enabled: controller.sourcePath.length > 0 && !controller.parsing
                    onClicked: controller.clear()
                }

                QGCLabel {
                    Layout.fillWidth: true
                    elide: Text.ElideMiddle
                    text: controller.sourcePath.length > 0
                          ? controller.sourcePath.replace(/.*[/\\]/, "")
                          : qsTr("No ULog selected")
                }
            }

            ProgressBar {
                Layout.fillWidth: true
                visible: controller.parsing
                from: 0
                to: 1
                value: controller.parseProgress
            }

            QGCLabel {
                Layout.fillWidth: true
                visible: controller.errorMessage.length > 0
                text: controller.errorMessage
                color: qgcPal.colorRed
                wrapMode: Text.WordWrap
            }

            Rectangle {
                Layout.fillWidth: true
                Layout.preferredHeight: summaryGrid.implicitHeight + ScreenTools.defaultFontPixelHeight
                visible: controller.ready
                color: qgcPal.windowShade
                radius: ScreenTools.defaultFontPixelWidth * 0.5

                GridLayout {
                    id: summaryGrid
                    anchors.fill: parent
                    anchors.margins: ScreenTools.defaultFontPixelHeight * 0.5
                    columns: 4
                    columnSpacing: ScreenTools.defaultFontPixelWidth * 2
                    rowSpacing: ScreenTools.defaultFontPixelHeight * 0.25

                    QGCLabel { text: qsTr("GPS source"); font.bold: true }
                    QGCLabel { text: controller.gpsSource }
                    QGCLabel { text: qsTr("Source rate"); font.bold: true }
                    QGCLabel { text: qsTr("%1 Hz").arg(controller.sourceRateHz.toFixed(2)) }

                    QGCLabel { text: qsTr("Flight duration"); font.bold: true }
                    QGCLabel { text: qsTr("%1 s").arg(controller.durationSeconds.toFixed(1)) }
                    QGCLabel { text: qsTr("Output rows"); font.bold: true }
                    QGCLabel { text: controller.rowCount.toLocaleString() }
                }
            }

            QGCLabel {
                Layout.fillWidth: true
                visible: controller.warningMessage.length > 0
                text: controller.warningMessage
                color: qgcPal.colorOrange
                wrapMode: Text.WordWrap
            }

            QGCLabel {
                visible: controller.ready
                text: qsTr("Preview")
                font.bold: true
            }

            Rectangle {
                Layout.fillWidth: true
                Layout.fillHeight: true
                visible: controller.ready
                color: qgcPal.window
                border.color: qgcPal.text
                border.width: 1

                ColumnLayout {
                    anchors.fill: parent
                    anchors.margins: ScreenTools.defaultFontPixelWidth
                    spacing: 0

                    RowLayout {
                        Layout.fillWidth: true

                        Repeater {
                            model: [qsTr("Auto"), qsTr("Waypoint"), qsTr("GPS TOW"),
                                    qsTr("Latitude"), qsTr("Longitude"), qsTr("Altitude")]

                            QGCLabel {
                                required property var modelData
                                Layout.fillWidth: true
                                Layout.preferredWidth: 1
                                text: modelData
                                font.bold: true
                                horizontalAlignment: Text.AlignHCenter
                            }
                        }
                    }

                    Rectangle {
                        Layout.fillWidth: true
                        Layout.preferredHeight: 1
                        color: qgcPal.text
                    }

                    ListView {
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        clip: true
                        model: controller.previewRows

                        delegate: RowLayout {
                            id: previewRow

                            required property var modelData
                            width: ListView.view.width

                            Repeater {
                                model: [previewRow.modelData.automatic, previewRow.modelData.waypoint,
                                        previewRow.modelData.gpsTime, previewRow.modelData.latitude,
                                        previewRow.modelData.longitude, previewRow.modelData.altitude]

                                QGCLabel {
                                    required property var modelData
                                    Layout.fillWidth: true
                                    Layout.preferredWidth: 1
                                    text: modelData
                                    horizontalAlignment: Text.AlignHCenter
                                }
                            }
                        }
                    }
                }
            }

            QGCFileDialog {
                id: openDialog
                title: qsTr("Open PX4 ULog")
                folder: QGroundControl.settingsManager.appSettings.logSavePath
                nameFilters: [qsTr("PX4 ULog Files (*.ulg *.ULG)")]
                onAcceptedForLoad: (file) => controller.loadULog(file)
            }

            QGCFileDialog {
                id: saveDialog
                title: qsTr("Save Competition Data")
                folder: QGroundControl.settingsManager.appSettings.logSavePath
                nameFilters: [qsTr("ASCII CSV Files (*.csv)")]
                defaultSuffix: "csv"
                onAcceptedForSave: (file) => controller.exportAscii(file)
            }

            Connections {
                target: controller

                function onExportFinished(filePath, ok, errorMessage) {
                    QGroundControl.showMessageDialog(
                                root,
                                qsTr("Competition Data Export"),
                                ok ? qsTr("Saved %1 rows to:\n%2").arg(controller.rowCount).arg(filePath)
                                   : errorMessage)
                }
            }
        }
    }
}
