
# 🚗 Detección de señales usando YOLO

Este paquete contiene el nodo de visión por computadora encargado de detectar 
señales de tránsito. Utiliza un modelo YOLOv8n entrenado.

## 🧠 1. El archivo 'best.pt'
El archivo `best.pt` ubicado en `src/demo/demo/` representa los pesos de la 
red neuronal entrenada en Google Colab. Es el "cerebro" que permite al nodo 
reconocer las 5 clases de señales (stop, escuela, giro, derrape, kilómetros).

---

## ⚙️ 2. Instalación de Dependencias (Ubuntu 24.04 Noble)
Debido a las restricciones de entorno de Python en esta versión, se deben 
instalar las librerías con los siguientes comandos:

**A. Dependencias de ROS 2:**
```bash
sudo apt update && sudo apt install ros-jazzy-cv-bridge
```

**B. Librerías de IA (Importante: NumPy < 2.0):**
```bash
pip install ultralytics opencv-python "numpy<2" --break-system-packages
```
---

## 🔑 3. Permisos de Ejecución
Asegúrate de que el script tenga permisos de ejecución antes de intentar correrlo:
```bash
chmod +x src/demo/demo/demo_seniales.py
```

---

## 🛠️ 4. Compilación del Workspace (CRÍTICO)
Para evitar la generación de archivos fuera de lugar y mantener la integridad 
del proyecto, la compilación DEBE hacerse siempre desde la raíz del workspace.

**Ruta correcta:** `/C-ROS-Automodel_Car/AutoModelRasp_ws`

**Instrucciones de compilación:**
1. Ve a la raíz:
   `cd ~/C-ROS-Automodel_Car/AutoModelRasp_ws`

2. (Opcional) Limpia compilaciones previas si hay errores de directorios:
   ```bash
   rm -rf build/ install/ log/
   ```
4. Compila con enlaces simbólicos para Python:
   ```bash
   colcon build --symlink-install
   ```

---

## 🚀 5. Ejecución y Debugging
Para lanzar el nodo y visualizar la detección en tiempo real:

1. Terminal del Nodo:
   ```bash
   source install/setup.bash
   ros2 run demo demo_seniales
   ```

3. Terminal de Visualización (RQT):
  En una terminal nueva ejecutar el comando para abrir eel rqt:
   ```bash
   rqt
   ```
   y configurar lso parámetros de visualización para poder ver la imagen generada.
