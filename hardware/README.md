# ⚙️ Diseño de Hardware y CAD

Este directorio almacena todos los diseños mecánicos, modificaciones del chasis del AutoModelCar y las piezas personalizadas para impresión 3D.

## 🗂️ Organización sugerida

Para mantener el orden en los diseños de C-ROS, por favor estructura tus archivos así:

* **`/piezas_3d`**: Archivos `.stl` o `.3mf` listos para ser enviados a la impresora 3D (soportes de cámara, monturas para el Lidar, etc.).
* **`/fuentes_cad`**: Los archivos de diseño originales y editables. 
  * Se prefieren proyectos de **FreeCAD** (`.FCStd`) para piezas paramétricas y estructurales.
  * Se pueden incluir proyectos de **Blender** (`.blend`) si se requiere modelado orgánico o renderizado de la carrocería.

> **⚠️ Nota de control de versiones:** Trata de subir solo las versiones finales o estables de los archivos CAD, ya que los archivos binarios grandes pueden volver lento el repositorio si se modifican con demasiada frecuencia.
