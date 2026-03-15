# 🚗 C-ROS AutoModelCar

Repositorio oficial del **AutoModelCar**, desarrollado por el equipo **C-ROS** perteneciente a la asociación **OpenSoruce-UPIITA**.

En el repositorio se encuentra el trabajo realizado para la participación en la competencia AutoModelCar 2026 organizada por la Federación Mexicana de Robótica, tando la parte del software desarrolaldo en ROS2 Jazzy, la parte del hardware usando herramientas como Blender y FreeCAD, y la programación de un microcontrolador STM32.

## 🗂️ Estructura del Repositorio

Para evitar conflictos de versiones y separar las capas de control, el proyecto se divide en las siguientes áreas:

* **`AutoModelRasp_ws/`**: Workspace de ROS2. Contiene todos los paquetes y nodos de alto nivel que se ejecutan en la computadora principal (visión, conducción autónoma, mapeo, etc.).
* **`stm_firmware/`**: Proyectos y código de bajo nivel (C/C++) para la placa STM32. Incluye la versión estable actual encargada del control de motores y lectura directa de hardware.
* **`hardware/`**: Archivos de diseño mecánico (CAD) para el chasis, soportes y piezas para impresión 3D.
* **`documentacion/`**: Hojas de datos, diagramas de conexión electrónica, configuraciones y manuales del proyecto.

## 💻 Hardware Principal
EL proyecto está pensado para ejecutarse con ciertas características de hardware como lo son: 
* **Unidad de Procesamiento:** Raspberry Pi 5.
* **Control de Motores:** Microcontrolador STM32.
* **Sensores:** Cámara de Profundidad y Lidar.

## 🚀 Cómo empezar a trabajar
1. Clona este repositorio en tu máquina local.
2. Si vas a trabajar en ROS, navega al workspace: `cd AutoModelRasp_ws`
3. Compila los paquetes usando: `colcon build` (Asegúrate de no subir las carpetas generadas al repositorio).

> **⚠️ IMPORTANTE PARA DESARROLLADORES:** Antes de crear una rama o escribir código, es OBLIGATORIO leer el archivo `CONTRIBUTING.md` para conocer nuestra metodología de trabajo y reglas de integración.
