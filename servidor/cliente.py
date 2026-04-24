import sys
import socket
import json
from PySide6.QtWidgets import QApplication, QMainWindow, QMessageBox
from PySide6.QtCore import Qt

# ¡Importamos tu interfaz compilada!
from interfaz_ui import Ui_MainWindow

class RobotGamepadUI(QMainWindow, Ui_MainWindow):
    def __init__(self):
        super().__init__()
        
        # Inicializamos la interfaz compilada
        self.setupUi(self)

        # Variables de red y estado del robot
        self.s = None
        self.conectado = False
        
        # Variables lógicas de movimiento
        self.current_dir_dc = 0
        self.current_dir_servo = 1500

        self.setup_signals()
        self.habilitar_controles(False)
        self.setFocusPolicy(Qt.StrongFocus) # Necesario para leer el teclado

    def setup_signals(self):
        # Conexión
        self.btn_conectar.clicked.connect(self.conectar_servidor)

        # Ajustes
        self.slider_speed.valueChanged.connect(self.actualizar_velocidad)
        self.check_stop.stateChanged.connect(self.enviar_estado)
        self.combo_turn.currentIndexChanged.connect(self.enviar_estado)

        # Eventos de los botones WASD en pantalla (Mantener presionado y Soltar)
        self.btn_w.pressed.connect(self.iniciar_adelante)
        self.btn_w.released.connect(self.detener_motor)
        
        self.btn_s.pressed.connect(self.iniciar_atras)
        self.btn_s.released.connect(self.detener_motor)
        
        self.btn_a.pressed.connect(self.iniciar_izquierda)
        self.btn_a.released.connect(self.centrar_direccion)
        
        self.btn_d.pressed.connect(self.iniciar_derecha)
        self.btn_d.released.connect(self.centrar_direccion)

    def habilitar_controles(self, estado):
        self.group_ajustes.setEnabled(estado)
        self.group_control.setEnabled(estado)

    def actualizar_velocidad(self):
        self.lbl_speed_val.setText(f"{self.slider_speed.value()}%")
        # Si el motor está en movimiento (dir_dc != 0), aplicamos el nuevo slider en tiempo real
        if self.current_dir_dc != 0:
            self.enviar_estado()

    # --- LÓGICA DE MOVIMIENTO ---
    def iniciar_adelante(self):
        self.current_dir_dc = 1
        self.check_stop.setChecked(False)
        self.btn_w.setDown(True) # Efecto visual de botón hundido
        self.enviar_estado()

    def iniciar_atras(self):
        self.current_dir_dc = 2
        self.check_stop.setChecked(False)
        self.btn_s.setDown(True)
        self.enviar_estado()

    def detener_motor(self):
        self.current_dir_dc = 0
        self.check_stop.setChecked(True) # Freno automático
        self.btn_w.setDown(False)
        self.btn_s.setDown(False)
        self.enviar_estado()

    def iniciar_izquierda(self):
        self.current_dir_servo = 1740
        self.combo_turn.setCurrentIndex(2) # Direccional izquierda automática
        self.btn_a.setDown(True)
        self.enviar_estado()

    def iniciar_derecha(self):
        self.current_dir_servo = 1110
        self.combo_turn.setCurrentIndex(1) # Direccional derecha automática
        self.btn_d.setDown(True)
        self.enviar_estado()

    def centrar_direccion(self):
        self.current_dir_servo = 1500
        self.combo_turn.setCurrentIndex(0) # Apaga direccionales
        self.btn_a.setDown(False)
        self.btn_d.setDown(False)
        self.enviar_estado()

    # --- INTERCEPCIÓN DEL TECLADO FÍSICO ---
    def keyPressEvent(self, event):
        if not self.conectado or event.isAutoRepeat():
            return

        key = event.key()
        if key == Qt.Key_W or key == Qt.Key_Up:
            self.iniciar_adelante()
        elif key == Qt.Key_S or key == Qt.Key_Down:
            self.iniciar_atras()
        elif key == Qt.Key_A or key == Qt.Key_Left:
            self.iniciar_izquierda()
        elif key == Qt.Key_D or key == Qt.Key_Right:
            self.iniciar_derecha()

    def keyReleaseEvent(self, event):
        if not self.conectado or event.isAutoRepeat():
            return

        key = event.key()
        if key in (Qt.Key_W, Qt.Key_Up, Qt.Key_S, Qt.Key_Down):
            self.detener_motor()
        elif key in (Qt.Key_A, Qt.Key_Left, Qt.Key_D, Qt.Key_Right):
            self.centrar_direccion()

    # --- RED Y COMUNICACIÓN ---
    def conectar_servidor(self):
        if self.conectado:
            if self.s: self.s.close()
            self.conectado = False
            self.lbl_estado.setText("Desconectado")
            self.lbl_estado.setStyleSheet("color: red; font-weight: bold;")
            self.btn_conectar.setText("Conectar")
            self.habilitar_controles(False)
            return

        try:
            ip = self.ip_input.text()
            puerto = int(self.port_input.text())
            self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.s.connect((ip, puerto))
            self.s.sendall(b"SOY_EL_USUARIO\n")
            
            self.conectado = True
            self.lbl_estado.setText("ONLINE")
            self.lbl_estado.setStyleSheet("color: green; font-weight: bold;")
            self.btn_conectar.setText("Desconectar")
            self.habilitar_controles(True)
            self.setFocus() # Asegurar que el teclado funcione de inmediato
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Fallo al conectar:\n{e}")

    def enviar_estado(self):
        if not self.conectado or not self.s:
            return

        # Construir el paquete a enviar
        payload = {
            "dir_dc": self.current_dir_dc,
            "speed_dc": self.slider_speed.value() if self.current_dir_dc != 0 else 0,
            "dir_servo": self.current_dir_servo,
            "stop_lights": 1 if self.check_stop.isChecked() else 0,
            "turn_signals": self.combo_turn.currentIndex()
        }

        try:
            mensaje = json.dumps(payload) + "\n"
            self.s.sendall(mensaje.encode('utf-8'))
        except Exception as e:
            print(f"Error de red: {e}")
            self.conectar_servidor() # Fuerzo desconexión si hay error

    def closeEvent(self, event):
        if self.s:
            self.s.close()
        event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    ventana = RobotGamepadUI()
    ventana.show()
    sys.exit(app.exec_())