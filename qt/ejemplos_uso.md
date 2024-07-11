# EJEMPLOS DE USO
[Fuente](https://doc.qt.io/qt-6/moc.html) <br>
 ### 1. Si quieres hacer un ENUM:

````
class MyClass : public QObject
{
    Q_OBJECT
    Q_PROPERTY(Priority priority READ priority WRITE setPriority)

public:
    enum Priority { High, Low, VeryHigh, VeryLow };
    Q_ENUM(Priority)

    MyClass(QObject *parent = 0);
    ~MyClass();

    void setPriority(Priority priority) { m_priority = priority; }
    Priority priority() const { return m_priority; }

private:
    Priority m_priority;
};
````

###  2. Información que quieras añadir
````
class MyClass : public QObject
{
    Q_OBJECT
    Q_CLASSINFO("Author", "Oscar Peterson")
    Q_CLASSINFO("Status", "Active")

public:
    MyClass(QObject *parent = 0);
    ~MyClass();
};
````
###  3. Herencia multiple
```
// correct
class SomeClass : public QObject, public OtherClass
{
    ...
};
````

### 4. Los punteros de función no pueden ser parámetros de señal o de ranura
En la mayoría de los casos en los que se consideraría el uso de punteros de función como parámetros de señal o de ranura, creemos que la herencia es una mejor alternativa.
A continuación, se muestra un ejemplo de sintaxis ilegal:
```
class SomeClass : public QObject
{
    Q_OBJECT

public slots:
    void apply(void (*apply)(List *, void *), char *); // WRONG
};
`````
Puedes solucionar esta restricción de la siguiente manera:
````
typedef void (*ApplyFunction)(List *, void *);

class SomeClass : public QObject
{
    Q_OBJECT

public slots:
    void apply(ApplyFunction, char *);
};
````
A veces puede ser incluso mejor reemplazar el puntero de función con herencia y funciones virtuales.






























