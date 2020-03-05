import QtQuick 2.7
import QtQuick.Controls 2.0
import QtQuick.Layouts 1.0
import OpenGL 1.0
import QtQuick.Dialogs 1.0

ApplicationWindow {
    visible: true
    width: 1200
    height: 700
    id: window

    DropArea // external drag & drop
    {
        anchors.fill: parent
        onDropped: UiManager.handleDrop(drop.text);
    }



    /*************************************
     *              MAP VIEW             *
     ************************************/

    OpenGLWindow
    {
        width: 1200
        height: 700

        id: map
        objectName: "map"

        focus: true
        Keys.onPressed:
        {
            if(event.key==86)
                toggleInterfaceVisibility();
            else
                UiManager.handleKey(event.key)
        }

        onFocusChanged: map.focus = true
        onUpdated: infoField.text = fps;

        MouseArea
        {
            hoverEnabled: true
            anchors.fill: parent
            onWheel:
            {
                var scale = wheel.angleDelta.y / 1200;
                map.handleScale(scale, mouseX, mouseY);
            }
            acceptedButtons: "NoButton"

        }


        SequentialAnimation on t {
            NumberAnimation { to: 1; duration: 2500; easing.type: Easing.InQuad }
            NumberAnimation { to: 0; duration: 2500; easing.type: Easing.OutQuad }
            loops: Animation.Infinite
            running: true

        }

        Text
        {
            anchors.right: parent.right
            id: infoField
            color: "white"
        }


        Text
        {
            anchors.right: parent.right
            anchors.bottom: parent.bottom
            id: infoFooter
            color: "white"
        }

        onZoomChanged: updateMapInfo();
        onOffsetChanged: updateMapInfo();

        function toggleInterfaceVisibility()
        {
            var v = true;
            if(simControlBar.visible==true)
                v = false;

            simControlBar.visible = v;
            toolBar.visible = v;
            infoField.visible = v;
            infoFooter.visible = v;
        }

        function updateMapInfo()
        {
            var text = "offset: " + parseInt(offset.x) + " / ";
            text += + parseInt(offset.y) + "\t";
            text += "zoom: " + zoom.toFixed(2);

            infoFooter.text = text;
        }


    }

    /*************************************
     *              TOOL BAR             *
     ************************************/
    Rectangle
    {
        visible: true
        id: toolBar
        height: window.height
        width: 50
        color: "#1c1c1c"

        Column
        {
            spacing: 1
            ToolItem
            {
                id: saveButton
                text: "Save"

                FileDialog {
                    id: saveDialog
                    visible: false
                    title: "Save"
                    folder: shortcuts.home

                    selectExisting: false
                    onAccepted:
                    {
                        var path = saveDialog.fileUrls;
                        var saved = UiManager.saveScenario(path);
                        saveButton.handleResult(saved);
                    }
                }

                onClicked: {saveDialog.visible = true;}

            }
            ToolItem
            {
                id: loadButton
                text: "Load"


                FileDialog {
                    id: loadDialog
                    visible: false
                    title: "Save"
                    folder: shortcuts.home

                    selectExisting: true
                    onAccepted:
                    {
                        var path = loadDialog.fileUrls;
                        var loaded = UiManager.loadScenario(path);
                        loadButton.handleResult(loaded);
                    }
                }

                onClicked: {loadDialog.visible = true;}
            }
            ToolItem
            {
                id: clearButton
                text: "Clear"
                onClicked: {UiManager.clear();}
            }
            ToolItem
            {
                id: exportButton
                text: "EPS"
                onClicked: {UiManager.exportScreenshot();}
            }
        }
    }

    Rectangle
    {
        id: simControlBar
        height: 50
        width: 600
        color: "#1c1c1c"

        anchors.horizontalCenter: parent.horizontalCenter

        Row
        {
            ToolItem
            {
                id: startButton
                text: "Start"
                onClicked:
                {
                    UiManager.start();
                }
            }

            ToolItem
            {
                id: stopButton
                text: "Stop"
                onClicked:
                {
                    UiManager.stop();
                }
            }

            ToolItem
            {
                id: stepButton
                text: "Step"
                onClicked:
                {
                    UiManager.step();
                }
            }
            Column
            {
                width: 100
                Label
                {
                    id: eventCounterLabel
                    color: "white"
                }
                Label
                {
                    id: simTimeLabel
                    color: "white"
                }
                Label
                {
                    id: simDurationLabel
                    color: "white"
                }
            }
            Slider
            {
                from: 1
                to: 100
                value: 10

                onValueChanged: UiManager.setTimeScaleFactor(value)
            }


        }

        Component.onCompleted:
        {
            UiManager.updated.connect(onUpdated);
            map.toggleInterfaceVisibility();
        }

        function getTimeDisplayString(_time_s)
        {
            var time_h = Math.floor(_time_s/3600);
            var remainder_h = _time_s % 3600;

            var time_m = Math.floor(remainder_h/60);
            var time_s = (remainder_h % 60).toFixed(2);

            var result = "";
            if(time_h>0)
                result += time_h + " h ";
            if(time_m>0)
                result += time_m + " m ";
            if(time_s>0)
                result += time_s + " s ";

            return result;
        }

        function onUpdated(_events, _scheduled, _simTime_s, _simDuration_s)
        {
            eventCounterLabel.text = _events + " / " + _scheduled + " Events";
            simTimeLabel.text = getTimeDisplayString(_simTime_s);
            simDurationLabel.text = getTimeDisplayString(_simDuration_s);
        }
    }


}
