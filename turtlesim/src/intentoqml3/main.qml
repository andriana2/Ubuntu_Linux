import QtQuick 2.15
import QtQuick.Window 2.15

Window {

    maximumHeight: 500
    maximumWidth:  600
    minimumHeight: maximumHeight
    minimumWidth:  maximumWidth
    visible: true
    title:   qsTr("joystick")

    property int value_x: 10
    property int value_y: 10
    property bool value : false


    Rectangle{
        id: udrl2
        x: 50
        y: 300
        width: 132
        height: 112

        Image {
            id: upper2
            x:  60
            y:  10
            source: "/images/up2.png"
            MouseArea{
                anchors.fill: parent
                onPressed:{
                    upper.opacity = 0.5
                    _rosNode.buttonCallback(10)
                }
                onReleased: upper.opacity  = 1.0
            }
        }

        Image {
            id: down2
            x: 60
            y: 90
            source: "/images/down2.png"
            MouseArea{
                anchors.fill: parent
                onPressed:{
                    down.opacity  = 0.5
                    _rosNode.buttonCallback(11)
                }
                onReleased: down.opacity  = 1.0
            }
        }
    }

    Rectangle{
        id: udrl
        x: 50
        y: 150
        width: 132
        height: 112

        Image {
            id: upper
            x:  50
            y:  0
            source: "/images/up.png"
            MouseArea{
                anchors.fill: parent
                onPressed:{
                    upper.opacity = 0.5
                    _rosNode.buttonCallback(0)
                }
                onReleased: upper.opacity  = 1.0
            }
        }

        Image {
            id: down
            x: 50
            y: 80
            source: "/images/down.png"
            MouseArea{
                anchors.fill: parent
                onPressed:{
                    down.opacity  = 0.5
                    _rosNode.buttonCallback(1)
                }
                onReleased: down.opacity  = 1.0
            }
        }


        Image {
            id: left
            x:  0
            y:  40
            source: "/images/giro_izq.png"
            MouseArea{
                anchors.fill: parent
                onPressed:  {
                    left.opacity = 0.5
                    _rosNode.buttonCallback(2)
                }
                onReleased: left.opacity = 1.0
            }
        }

        Image {
            id: right
            x:  100
            y:  40
            opacity: 1.0
            source: "images/giro_der.png"
            MouseArea{
                anchors.fill: parent
                onPressed: {
                    right.opacity  = 0.5
                    _rosNode.buttonCallback(3)
                }
                onReleased: right.opacity  = 1.0
            }
        }
    }

    Text{
        id: name
        x:275; y:0
        text: _rosNode.name
        font.family: "Arial"
        font.bold: true
        font.pointSize: 14
    }
}
