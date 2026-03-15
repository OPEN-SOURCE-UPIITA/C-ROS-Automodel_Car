# 🧠 Firmware del STM32F407VET6

Aquí encontrarás los programas para controlar el microcontrolador de la placa base. Incluye desde las pruebas básicas de hardware hasta el firmware oficial de comunicación con la Raspberry Pi (`stm_com_rasp`).

**⚡ IMPORTANTE: Archivos .hex listos para usar**
Para agilizar las pruebas físicas y no depender del IDE, el archivo ejecutable ya compilado del proyecto principal se encuentra disponible y listo para flashear. *(Nota: Los proyectos de prueba requerirán ser compilados manualmente).*

---

## 🛠️ Requisitos
* **STM32CubeIDE:** Entorno oficial para editar y compilar el código C.
* **stm32flash:** Herramienta de terminal para Linux usada para cargar el firmware por puerto serial.

---

## 🚀 Instrucciones de Uso

### Paso 1: Compilar el código (Opcional si ya tienes el .hex)
1. Abre **STM32CubeIDE**.
2. Ve a `File` -> `Open Projects from File System...`, haz clic en *Directory* y selecciona la carpeta del proyecto que deseas probar (ej. `MotorDC`).
3. **Verifica la generación del .hex:** En el menú superior, ve a `Project` -> `Properties`. En la ventana que se abre, navega a `C/C++ Build` -> `Settings` -> `MCU/MPU Post build outputs` y asegúrate de que esté marcada la casilla **Convert to Intel Hex file (-O ihex)**.
4. Haz clic en el ícono del **Martillo 🔨** (Build Debug) en la barra de herramientas superior.
5. Verifica en la consola inferior que el proceso termine con el mensaje `Build Finished. 0 errors`. Esto generará la carpeta `Debug/` con tu archivo `.hex`.

### Paso 2: Cargar el código al Microcontrolador (Flasheo)
1. **Conexión:** Conecta el robot a tu computadora utilizando el cable USB-C (asegúrate de usar el **puerto central**).
2. **Modo Bootloader (Hardware):** Para que la placa permita sobreescribir su memoria, debes ponerla en modo de programación siguiendo esta secuencia exacta:
   * Mantén presionado el **Custom button** (también marcado como **BOOT0**, ubicado junto a los pines GPIO).
   * Sin soltar el botón anterior, presiona y suelta el botón **RESET**.
3. **Preparar la terminal:** Abre una terminal de Linux directamente en la carpeta donde se encuentra tu archivo `.hex` (generalmente dentro de la subcarpeta `Debug/`).
4. **Ejecutar el comando:** Ejecuta la siguiente instrucción para flashear la placa (recuerda cambiar `NombreArchivo.hex` por el nombre real de tu archivo):

   ```bash
   stm32flash -w NombreArchivo.hex -v -g 0x0 -R /dev/ttyACM0
   ```
   
5. **Finalizar:** Una vez que la terminal indique que el programa se ha cargado con éxito, suelta el **Custom button** (presiona **RESET** si es necesario)

## 📚 Nuevo proyecto
Para la creación de nuevos programas consultar el pdf guía: _Guía programación STM32_, ubicado en la carpeta de `documentacion`, dónde se detalla cómo funciona la creación de un proyecto usando las herramientas oficiales de STM.
