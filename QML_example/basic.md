## EJEMPLO BASICO
````c
import QtQuick

Window {
  id: root

  width: 640
  height: 480
  visible: true
  title: qsTr("Hello World") //nombre arriba en el ejecutable

  Rectangle {
    id: button1

    property color baseColor: "red"

    anchors { //marco para que no se mueva
      bottom: parent.verticalCenter
      horizontalCenter: parent.horizontalCenter
      bottomMargin: 20
    }

    width: 150
    height: 50

    color: if (buttonMouseArea1.containsPress) { //si presiono cambia de color 
             return Qt.darker(baseColor)
           } else if (buttonMouseArea1.containsMouse) {//Si estoy enciama con el raton cambia de color
             return Qt.lighter(baseColor)
           } else {
             return baseColor
           }

    Text {
      id: buttonText1

      anchors.centerIn: parent

      text: "CLICK ME!!!"
    }

    MouseArea {
      id: buttonMouseArea1

      anchors.fill: parent

      hoverEnabled: true

      onClicked: {
        button1.baseColor = "orange"
      }
    }
  }

  Rectangle {

    property color baseColor: "#00FF00"

    anchors {
      top: parent.verticalCenter
      horizontalCenter: parent.horizontalCenter
      topMargin: 20
    }

    width: 150
    height: 50

    color: if (buttonMouseArea2.containsPress) {
             return Qt.darker(baseColor)
           } else if (buttonMouseArea2.containsMouse) {
             return Qt.lighter(baseColor)
           } else {
             return baseColor
           }

    Text {
      id: buttonText2

      anchors.centerIn: parent

      text: "Close application"
    }

    MouseArea {
      id: buttonMouseArea2

      anchors.fill: parent

      hoverEnabled: true

      onClicked: {
        Qt.quit()
      }
    }
  }

  Connections {
    target: buttonMouseArea2

    function onClicked() {
      console.log("Button 2 clicked, exiting application")
    }
  }
}

````
