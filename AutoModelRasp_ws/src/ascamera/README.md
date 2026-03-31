# ascamera: Driver de Cámara RGBD

**Este paquete permite la integración de la cámara de profundidad en ROS 2 usando una camara modelo hpc60c .**

![RViz Preview](rviz_preview.png)

##Este repositorio se puede encontrar en:

```bash
git clone https://github.com/j0z3ph/ascam_ros2.git

```
Compila el paquete desde tu Workspace:

```bash
cd ~/tuWorkspace_ws/
colcon build --symlink-install --packages-select ascamera

```

## 3. Instalar reglas udev

Configura los permisos USB para no requerir sudo al usar la cámara:

```bash
cd ~/tuWorkspace_ws/src/ascamera/scripts
sudo bash create_udev_rules.sh

```

*(Desconecta y vuelve a conectar la cámara después de este paso).*

## 4. Actualizar ruta de configuración (Importante)

El driver requiere que especifiques manualmente la ruta del archivo de configuración `.json`.

1. **Obtén la ruta de tu archivo de configuración:**

```bash
cd ~/tuWorkspace_ws/src/ascam_ros2/ascamera/configurationfiles/hp60c_v2_00_20230704_configEncrypt.json
pwd
```


2. **Edita el archivo de lanzamiento:**
Entra a la carpeta launch y abre el archivo:
```bash
cd ~/tuWorkspace/src/ascamera/launch
nano hp60c.launch.py

```


3. **Pega la ruta:**
Busca la variable `confi_path` y reemplaza su contenido con la ruta completa que copiaste en el paso 1.

```bash
{"confiPath": "/home/tu_usr/tuWorkSpace_ws/src/ascamera/configurationfiles"},
```

## 5. Ejecución (Launch)

Carga el entorno y lanza el driver:

```bash
source /tuWorkspace_ws/install/setup.bash
ros2 launch ascamera hp60c.launch.py

```

## 6. Visualización

Para ver las imágenes y la nube de puntos:

```bash
# Opción 1: RQT Image View
ros2 run rqt_image_view rqt_image_view

# Opción 2: RViz2 (Recomendado)
rviz2

```

### Configuración RViz

* **Frame ID:** `ascamera_hp60c_color_0`
* **Tópicos:** Agrega `/ascamera/images` o `/ascamera/depth`.
