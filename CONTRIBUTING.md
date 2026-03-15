# 🛠️ Metodología de Trabajo y Reglas de Contribución

Para garantizar la estabilidad del AutoModelCar y evitar problemas de integración al unir el código de diferentes miembros de C-ROS, todos los desarrolladores deben adherirse a la siguiente metodología.

## 1. La Regla de Oro: La rama `main` es sagrada
La rama `main` está protegida y siempre debe contener código 100% funcional. 
* **Prohibido:** Hacer `git push` directo a `main`.
* **Obligatorio:** Todo el código nuevo debe integrarse mediante un *Pull Request* (PR) y requiere aprobación para ser fusionado.

## 2. Nodos Core (Intocables)
Dentro de `AutoModelRasp_ws/src/`, existen nodos de hardware que ya son estables. **NO SE DEBEN MODIFICAR** sin autorización directa, ya que romperlos afecta a todos los demás paquetes:
* `comSTM` (Driver de comunicación con el STM32)
* `camera` (Driver de la cámara)
* `lidar` (Driver del Lidar)

## 3. Arquitectura de Paquetes ROS
Las nuevas funcionalidades deben desarrollarse dentro de sus respectivos paquetes para mantener la modularidad:
* **`paquete manual`**: Nodos de control por mando, teclado y rutinas de prueba.
* **`paquete conduccion`**: Nodos de seguimiento de carril, detección de señales y evasión de obstáculos.
* **`paquete estacionamiento`**: Nodos de mapeo y trazado de rutas para estacionarse.

## 4. Flujo de Desarrollo (El ciclo de vida del código)
Sigue estrictamente estos pasos para añadir código al proyecto:

1. **Trabaja en tu propia rama:** Crea una rama a partir de `main` con un nombre descriptivo de la tarea.
2. **Desarrolla localmente:** Escribe tu código y compila en tu computadora usando `colcon build` dentro de `AutoModelRasp_ws`. *(El `.gitignore` se encargará de no subir tus archivos de compilación).*
3. **Sincroniza antes de terminar:** Antes de hacer pruebas finales, actualiza tu rama con los últimos cambios aprobados en el repositorio:
   `git pull origin main`
   *(Resuelve los conflictos de código en tu computadora, no en el hardware del auto).*
4. **LA PRUEBA FÍSICA (Obligatoria):** Descarga tu rama actualizada directamente en la **Raspberry**. Compila el workspace y realiza una prueba física.
5. **Pull Request:** Solo si la prueba física es exitosa y el auto se comporta como se espera, abre un PR en GitHub hacia la rama `main`.

> **⚠️ Advertencia: Uso de la Raspberry**
> 
> La Raspberry es un entorno de validación de programas, no de desarrollo principal, así que si **durante la prueba de programas se modifica el código para solucionar errores, se debe hacer el commit en tu rama desde la Raspberry**.
>
> Al finalizar las pruebas es obligatorio:
> 1. Realizar el push de los últimos cambios de tu rama en Github.
> 2. Regresar el repositorio de la Raspberry a la rama principal
> 3. Asegurarse que el auto quede 100% funcional para el siguiente compañero que necesite trabajar.
