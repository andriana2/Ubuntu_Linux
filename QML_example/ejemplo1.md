## Q_PROPERTY
````c
Q_PROPERTY(tipo nombre
           READ métodoLectura
           WRITE métodoEscritura
           NOTIFY señalNotificacion
           RESET métodoReset
           DESIGNABLE bool
           SCRIPTABLE bool
           STORED bool
           USER bool)
````
### EJEMPLO DE USO
````c
class MiClase : public QObject
{
    Q_OBJECT
    Q_PROPERTY(int miValor READ obtenerMiValor WRITE establecerMiValor NOTIFY miValorCambiado)

public:
    explicit MiClase(QObject *parent = nullptr) : QObject(parent), m_valor(0) {}

    int obtenerMiValor() const { return m_valor; }
    void establecerMiValor(int valor) {
        if (m_valor != valor) {
            m_valor = valor;
            emit miValorCambiado(m_valor);  // Notifica sobre el cambio
        }
    }

signals:
    void miValorCambiado(int nuevoValor);

private:
    int m_valor;
};

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    MiClase objeto;

    // Conectar la señal miValorCambiado con un slot lambda para mostrar el nuevo valor
    QObject::connect(&objeto, &MiClase::miValorCambiado, [](int nuevoValor){
        qDebug() << "El valor ha cambiado a:" << nuevoValor;
    });

    objeto.establecerMiValor(10);
    qDebug() << "El valor actual es:" << objeto.obtenerMiValor();
    objeto.establecerMiValor(20);
    return a.exec();
}
````
### LA SALIDA
````C
El valor ha cambiado a: 10
El valor actual es: 10
El valor ha cambiado a: 20
````

