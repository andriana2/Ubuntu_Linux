cmake_minimum_required(VERSION 3.8)
project(intento3 VERSION 0.1 LANGUAGES CXX)

# Establece el estándar de C++ a C++17 y asegura que sea obligatorio.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Añade opciones de compilación adicionales si se usa GCC o Clang.
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Opción para controlar si se debe habilitar el soporte de ROS 2.
option(USE_ROS "Build with ROS2 support" ON)

# Comprueba si estamos en un entorno con ROS 2 (usando la variable de entorno ROS_DISTRO).
if(DEFINED ENV{ROS_DISTRO})
    message(STATUS "Entorno ROS detectado: $ENV{ROS_DISTRO}")
    set(USE_ROS ON)
else()
    message(STATUS "No se detectó entorno ROS. Compilando sin soporte para ROS.")
    set(USE_ROS OFF)
endif()

# Intenta encontrar Qt6 con el módulo Widgets.
find_package(Qt6 COMPONENTS Widgets QUIET)

# Si se encuentra Qt6, activa el soporte de Qt.
if(Qt6_FOUND)
    message(STATUS "Qt6 encontrado. Compilando con soporte para Qt.")
    set(USE_QT ON)
else()
    message(WARNING "Qt6 no encontrado. Compilando sin soporte para Qt.")
    set(USE_QT OFF)
endif()

# Dependencias de ROS 2.
if(USE_ROS)
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(geometry_msgs REQUIRED)

endif()

# Configuraciones relacionadas con Qt.
if(USE_QT)
    # Habilita la generación automática de código (MOC, RCC, UIC) para Qt.
    set(CMAKE_AUTOMOC ON)
    set(CMAKE_AUTORCC ON)
    set(CMAKE_AUTOUIC ON)

    # Establece el directorio de búsqueda para archivos .ui (interfaces de usuario).
    set(CMAKE_AUTOUIC_SEARCH_PATHS ${CMAKE_CURRENT_SOURCE_DIR}/ui)

    # Encuentra todos los archivos de cabecera, fuente, interfaces y recursos.
    file(GLOB HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h *.hpp)
    file(GLOB SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp *.c)
    file(GLOB UI_FILES ${CMAKE_CURRENT_SOURCE_DIR}/ui/*.ui)
    file(GLOB RESOURCES ${CMAKE_CURRENT_SOURCE_DIR}/resources/*.qrc)

    # Define el ejecutable con los archivos encontrados.
    add_executable(${PROJECT_NAME}
        ${HEADERS}
        ${SOURCES}
        ${UI_FILES}
        ${RESOURCES}
    )

    # Incluye el directorio de cabeceras en el proyecto.
    target_include_directories(${PROJECT_NAME} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    )

    # Añade las dependencias de ROS 2 si está habilitado.
    if(USE_ROS)
        ament_target_dependencies(${PROJECT_NAME}
            rclcpp
            std_msgs
            geometry_msgs
        )
    endif()

    # Enlaza la biblioteca de Widgets de Qt6.
    target_link_libraries(${PROJECT_NAME} 
        Qt6::Widgets
    )

    # Configura el proyecto como aplicación GUI en Windows.
    if(WIN32)
        set_target_properties(${PROJECT_NAME} PROPERTIES WIN32_EXECUTABLE TRUE)
    endif()

    # Configura el proyecto como un bundle en macOS.
    if(APPLE)
        set_target_properties(${PROJECT_NAME} PROPERTIES
            MACOSX_BUNDLE TRUE
            MACOSX_BUNDLE_GUI_IDENTIFIER com.example.${PROJECT_NAME}
            MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
            MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
        )
    endif()
endif()

# Instalación de archivos y objetivos.
if(USE_ROS)
    # Instala el ejecutable en el directorio de bibliotecas de ROS.
    install(TARGETS ${PROJECT_NAME}
        DESTINATION lib/${PROJECT_NAME}
    )

    # Instala los archivos de lanzamiento (launch files).
    install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}
    )

    # Crea el paquete de ament para ROS 2.
    ament_package()
else()
    # Si no se usa ROS pero se usa Qt, instala el ejecutable con Qt.
    if(USE_QT)
        install(TARGETS ${PROJECT_NAME}
            BUNDLE DESTINATION .
            LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
            RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        )
    else()
        # Si no hay soporte para ROS ni Qt, muestra una advertencia.
        message(WARNING "Ni ROS ni Qt están disponibles. No se construirán objetivos.")
    endif()
endif()

# Añade definiciones de compilación si se está utilizando ROS.
if(USE_ROS)
    target_compile_definitions(${PROJECT_NAME} PRIVATE USE_ROS)
endif()
