pragma ComponentBehavior: Bound

import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import QGroundControl
import QGroundControl.Controls
import QGroundControl.AnalyzeView

AnalyzePage {
    id: root

    readonly property var competitionColumnWidths: [90, 90, 150, 125, 125, 100, 90, 90, 90, 90, 90, 90]
    readonly property real competitionTableWidth: 1220

    pageComponent: pageComponent
    pageDescription: qsTr("Export a selected ARM-to-DISARM range as the competition 10 Hz, 12-column ASCII format using GPST and MSL altitude.")
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
                Layout.preferredHeight: rangeLayout.implicitHeight + ScreenTools.defaultFontPixelHeight
                visible: controller.armEvents.length > 0 || controller.disarmEvents.length > 0
                color: qgcPal.windowShade
                radius: ScreenTools.defaultFontPixelWidth * 0.5

                ColumnLayout {
                    id: rangeLayout
                    anchors.fill: parent
                    anchors.margins: ScreenTools.defaultFontPixelHeight * 0.5
                    spacing: ScreenTools.defaultFontPixelHeight * 0.4

                    GridLayout {
                        Layout.fillWidth: true
                        columns: 4
                        columnSpacing: ScreenTools.defaultFontPixelWidth

                        QGCLabel {
                            text: qsTr("Start ARM")
                            font.bold: true
                        }

                        QGCComboBox {
                            Layout.fillWidth: true
                            model: controller.armEvents
                            textRole: "label"
                            currentIndex: controller.selectedArmIndex
                            onActivated: (index) => controller.selectedArmIndex = index
                        }

                        QGCLabel {
                            text: qsTr("End DISARM")
                            font.bold: true
                        }

                        QGCComboBox {
                            Layout.fillWidth: true
                            model: controller.disarmEvents
                            textRole: "label"
                            currentIndex: controller.selectedDisarmIndex
                            onActivated: (index) => controller.selectedDisarmIndex = index
                        }
                    }

                    QGCLabel {
                        Layout.fillWidth: true
                        text: qsTr("Everything between these two boundaries is exported, regardless of intermediate ARM/DISARM cycles.")
                        wrapMode: Text.WordWrap
                    }
                }
            }

            Rectangle {
                Layout.fillWidth: true
                Layout.preferredHeight: summaryGrid.implicitHeight + ScreenTools.defaultFontPixelHeight
                visible: controller.armEvents.length > 0
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
                    QGCComboBox {
                        Layout.fillWidth: true
                        model: controller.gpsSourceOptions
                        textRole: "label"
                        currentIndex: controller.selectedGpsSourceIndex
                        enabled: !controller.parsing
                        onActivated: (index) => controller.selectedGpsSourceIndex = index
                    }
                    QGCLabel { text: qsTr("IMU source"); font.bold: true }
                    QGCLabel { text: controller.imuSource }

                    QGCLabel { text: qsTr("Attitude source"); font.bold: true }
                    QGCLabel { text: controller.attitudeSource }
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

                Flickable {
                    anchors.fill: parent
                    anchors.margins: ScreenTools.defaultFontPixelWidth
                    contentWidth: root.competitionTableWidth
                    contentHeight: height
                    flickableDirection: Flickable.HorizontalFlick
                    clip: true

                    ScrollBar.horizontal: ScrollBar {}

                    Column {
                        id: previewTable
                        width: root.competitionTableWidth
                        height: parent.height
                        spacing: 0

                        Row {
                            width: parent.width
                            height: ScreenTools.defaultFontPixelHeight * 2

                            Repeater {
                                model: [qsTr("Auto"), qsTr("Event"), qsTr("GPST"),
                                        qsTr("Latitude"), qsTr("Longitude"), qsTr("Altitude MSL"),
                                        qsTr("Ax"), qsTr("Ay"), qsTr("Az"),
                                        qsTr("Roll"), qsTr("Pitch"), qsTr("Yaw")]

                                QGCLabel {
                                    required property int index
                                    required property var modelData
                                    width: root.competitionColumnWidths[index]
                                    height: parent.height
                                    text: modelData
                                    font.bold: true
                                    horizontalAlignment: Text.AlignHCenter
                                    verticalAlignment: Text.AlignVCenter
                                }
                            }
                        }

                        Rectangle {
                            width: parent.width
                            height: 1
                            color: qgcPal.text
                        }

                        ListView {
                            width: parent.width
                            height: parent.height - y
                            clip: true
                            model: controller.previewRows

                            delegate: Row {
                                id: previewRow

                                required property var modelData
                                width: ListView.view.width
                                height: ScreenTools.defaultFontPixelHeight * 1.8

                                Repeater {
                                    model: [previewRow.modelData.automatic, previewRow.modelData.event,
                                            previewRow.modelData.gpst, previewRow.modelData.latitude,
                                            previewRow.modelData.longitude, previewRow.modelData.altitude,
                                            previewRow.modelData.ax, previewRow.modelData.ay,
                                            previewRow.modelData.az, previewRow.modelData.roll,
                                            previewRow.modelData.pitch, previewRow.modelData.yaw]

                                    QGCLabel {
                                        required property int index
                                        required property var modelData
                                        width: root.competitionColumnWidths[index]
                                        height: parent.height
                                        text: modelData
                                        horizontalAlignment: Text.AlignHCenter
                                        verticalAlignment: Text.AlignVCenter
                                    }
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
                nameFilters: [qsTr("ASCII Text Files (*.txt *.asc)")]
                defaultSuffix: "txt"
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
