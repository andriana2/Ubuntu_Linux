
# Explicación de etiquetas en QML (Qt Creator 6)

En **QML** (Qt Modeling Language), utilizado en **Qt Quick** para diseñar interfaces gráficas, las etiquetas definen los diferentes componentes que componen una UI (Interfaz de Usuario). A continuación se desglosa un código QML y se explica cómo funcionan las etiquetas y componentes en **Qt Creator 6**.

## 1. `Window`:
El componente principal es una **ventana** que contiene todos los demás elementos de la interfaz.

```qml
Window {
    width: 640
    height: 480
    visible: true
    title: qsTr("Hello World")
    color: "lightyellow"
}
```

- `width`, `height`: Definen las dimensiones de la ventana.
- `visible:` `true`: Hace la ventana visible cuando se ejecuta la aplicación.
- `title:` `qsTr("Hello World")`: Define el título de la ventana (la barra superior del sistema operativo).
- `color:` `"lightyellow"`: Establece el color de fondo de la ventana (en este caso, un amarillo claro).

## 2. `GridLayout`:
Dentro de la ventana, has usado un **GridLayout**. Es un layout que organiza los elementos en una cuadrícula (con filas y columnas).

```qml
GridLayout {
    anchors.fill: parent
    rows: 2
    columns: 2
    rowSpacing: 10
    columnSpacing: 10
}
```

- `anchors.fill: parent`: Esto ancla el `GridLayout` al tamaño completo de su contenedor principal (la ventana en este caso), ocupando todo el espacio disponible.
- `rows: 2`, `columns: 2`: Indica que habrá 2 filas y 2 columnas, creando un total de 4 celdas (2x2).
- `rowSpacing: 10`, `columnSpacing: 10`: Establece un espacio de 10 píxeles entre las filas y columnas.

Dentro de este `GridLayout`, has incluido 3 elementos `Item`. Los `Item` en QML son objetos básicos que actúan como contenedores, a los cuales se les puede añadir componentes visuales como `Rectangle` o `Flow`.

## 3. Primer `Item`:
El primer `Item` contiene un `Rectangle` que está posicionado usando anclajes.

```qml
Item {
    Layout.fillWidth: true
    Layout.fillHeight: true
    Rectangle {
        anchors {
            top: parent.verticalCenter;
            left: parent.horizontalCenter;
        }
        width: 100
        height: 75
        color: "darkred"
    }
}
```

- `Layout.fillWidth: true`, `Layout.fillHeight: true`: Estos atributos permiten que el `Item` y su contenido (el `Rectangle`) ocupen todo el espacio de la celda en el `GridLayout`.
- Dentro del `Item` hay un `Rectangle` que tiene un tamaño fijo (`width: 100` y `height: 75`) y un color rojo oscuro.
- Los **anclajes** (`anchors`) posicionan el rectángulo en el centro vertical y horizontal de su celda. Esto es opcional, pero en este caso asegura que el `Rectangle` no esté en la esquina superior izquierda por defecto.

## 4. Segundo `Item` con un `Flow`:
Este `Item` utiliza un **Flow** para distribuir visualmente varios elementos en una línea (flujo), ya sea horizontal o verticalmente.

```qml
Item {
    Layout.fillWidth: true
    Layout.fillHeight: true
    Flow {
        anchors.fill: parent
        spacing: 20
        Repeater {
            model: 5
            Rectangle {
                width: 50
                height: 50
                color: "green"
            }
        }
    }
}
```

- **Flow**: Es un layout que organiza elementos en filas o columnas, según el espacio disponible. En este caso, los elementos se colocan horizontalmente uno al lado del otro.
- `anchors.fill: parent`: Hace que el `Flow` llene completamente el `Item`.
- `spacing: 20`: Define un espacio de 20 píxeles entre los elementos en el flujo.
- **Repeater**: Genera dinámicamente varios elementos. El atributo `model: 5` crea 5 instancias de `Rectangle`, cada uno con tamaño 50x50 y color verde.
- De esta manera, dentro de este `Flow` tienes cinco cuadrados verdes distribuidos horizontalmente con un espaciado de 20 píxeles entre ellos.

## 5. Tercer `Item` con un `GridLayout` anidado:
Este `Item` contiene otro `GridLayout` dentro de él, creando una cuadrícula más pequeña con 2 filas y 2 columnas.

```qml
Item {
    Layout.fillHeight: true
    Layout.fillWidth: true
    GridLayout {
        anchors.fill: parent
        rows: 2
        columns: 2
        rowSpacing: 20
        columnSpacing: 20
        Rectangle {
            Layout.fillWidth: true
            Layout.fillHeight: true
            color: "black"
        }
        Rectangle {
            Layout.fillWidth: true
            Layout.fillHeight: true
            color: "red"
        }
        Rectangle {
            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.columnSpan: 2
            color: "magenta"
        }
    }
}
```

- `GridLayout`: Igual que el anterior, organiza los elementos en una cuadrícula.
- Dentro del `GridLayout` hay 3 `Rectangle`:
  - El primero es un rectángulo negro que ocupa una celda.
  - El segundo es un rectángulo rojo que ocupa otra celda.
  - El tercero es un rectángulo magenta, pero con `Layout.columnSpan: 2`, lo que significa que ocupa el ancho de **dos columnas**, abarcando toda la fila inferior.

## Resumen:

- **Layouts (GridLayout, Flow)**: Controlan cómo se distribuyen los elementos en la interfaz, en forma de cuadrícula o flujo.
- **Item**: Actúa como contenedor básico. No tiene apariencia visual a menos que le agregues componentes como `Rectangle`.
- **Rectangle**: Un componente visual que se usa para dibujar formas básicas.
- **Repeater**: Se utiliza para crear múltiples instancias de un componente repetidamente.
- **Anclajes (anchors)**: Sirven para posicionar elementos en relación con su contenedor o con otros elementos.

Este código crea una cuadrícula 2x2 donde los `Item` dentro de las celdas tienen diferentes layouts y distribuciones internas. Cada uno de los elementos tiene diferentes comportamientos y características visuales, como colores y tamaños.
