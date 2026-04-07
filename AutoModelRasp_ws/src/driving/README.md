# Se crea la carpeta para guardar la configuraciones
mkdir -p ~/C-ROS-Automodel_Car/AutoModelRasp_ws/src/driving/config

# Alias para guardar la calibración de la señal de STOP
echo "alias savesenial='ros2 param dump /detector_senales > ~/C-ROS-Automodel_Car/AutoModelRasp_ws/src/driving/config/calibracion_stop.yaml'" >> ~/.bashrc

# Alias para guardar la calibración de los carriles
echo "alias savecarril='ros2 param dump /detector_carril > ~/C-ROS-Automodel_Car/AutoModelRasp_ws/src/driving/config/calibracion_carril.yaml'" >> ~/.bashrc

# Alias para activar el nodo deteccion_senial con la configuración
echo "alias senialon='ros2 run driving detector_senales --ros-args --params-file ~/C-ROS-Automodel_Car/AutoModelRasp_ws/src/driving/config/calibracion_stop.yaml'" >> ~/.bashrc

# Alias para activar el nodo deteccion_carril con la configuración
echo "alias carrilon='ros2 run driving detector_carril --ros-args --params-file ~/C-ROS-Automodel_Car/AutoModelRasp_ws/src/driving/config/calibracion_carril.yaml'" >> ~/.bashrc

# Comando para que se actualice el comando en la terminal
source ~/.bashrc

# Comandos:
 savesenial - Guarda los datos del rqt para la siguiente configuración
 savecarril - Guarda los datos del rqt para la siguiente configuración
 senialon - Lanza el nodo deteccion_senial con la configuracion que guardamos
 carrilon - Lanza el nodo deteccion_carril con la configuracion que guardamos
