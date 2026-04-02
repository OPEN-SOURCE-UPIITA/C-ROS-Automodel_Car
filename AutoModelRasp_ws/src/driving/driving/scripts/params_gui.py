#!/usr/bin/env python3
#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                                QHBoxLayout, QLabel, QSlider, QGroupBox, 
                                QGridLayout, QPushButton, QSpinBox)
from PySide6.QtCore import Qt, QTimer
from std_msgs.msg import Int32MultiArray

class ParamSliderWidget(QWidget):
    """Widget individual para un parámetro HSV (H, S, o V)"""
    def __init__(self, name, min_val, max_val, default_val, parent=None):
        super().__init__(parent)
        self.name = name
        self.default_val = default_val
        
        layout = QHBoxLayout()
        
        # Etiqueta del nombre
        self.label_name = QLabel(name)
        self.label_name.setFixedWidth(40)
        
        # Slider
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setMinimum(min_val)
        self.slider.setMaximum(max_val)
        self.slider.setValue(default_val)
        self.slider.valueChanged.connect(self.update_value)
        
        # SpinBox para valor exacto
        self.spinbox = QSpinBox()
        self.spinbox.setMinimum(min_val)
        self.spinbox.setMaximum(max_val)
        self.spinbox.setValue(default_val)
        self.spinbox.valueChanged.connect(self.slider.setValue)
        
        # Etiqueta del valor actual
        self.label_value = QLabel(str(default_val))
        self.label_value.setFixedWidth(35)
        
        layout.addWidget(self.label_name)
        layout.addWidget(self.slider)
        layout.addWidget(self.spinbox)
        layout.addWidget(self.label_value)
        
        self.setLayout(layout)
    
    def update_value(self, value):
        self.label_value.setText(str(value))
        self.spinbox.blockSignals(True)
        self.spinbox.setValue(value)
        self.spinbox.blockSignals(False)
    
    def get_value(self):
        return self.slider.value()

class StopSignGUI(QMainWindow):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.setWindowTitle("Configurador de Señal STOP")
        self.setMinimumSize(600, 400)
        
        # Widget central
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        
        # Grupo para Rango Rojo 1 (0-10°)
        group1 = QGroupBox("Rango Rojo 1 (0° - 10°)")
        group1_layout = QGridLayout()
        
        self.lower1_h = ParamSliderWidget("H min", 0, 180, 0)
        self.lower1_s = ParamSliderWidget("S min", 0, 255, 100)
        self.lower1_v = ParamSliderWidget("V min", 0, 255, 100)
        self.upper1_h = ParamSliderWidget("H max", 0, 180, 10)
        self.upper1_s = ParamSliderWidget("S max", 0, 255, 255)
        self.upper1_v = ParamSliderWidget("V max", 0, 255, 255)
        
        group1_layout.addWidget(QLabel("Lower:"), 0, 0)
        group1_layout.addWidget(self.lower1_h, 0, 1)
        group1_layout.addWidget(self.lower1_s, 0, 2)
        group1_layout.addWidget(self.lower1_v, 0, 3)
        group1_layout.addWidget(QLabel("Upper:"), 1, 0)
        group1_layout.addWidget(self.upper1_h, 1, 1)
        group1_layout.addWidget(self.upper1_s, 1, 2)
        group1_layout.addWidget(self.upper1_v, 1, 3)
        
        group1.setLayout(group1_layout)
        
        # Grupo para Rango Rojo 2 (170-180°)
        group2 = QGroupBox("Rango Rojo 2 (170° - 180°)")
        group2_layout = QGridLayout()
        
        self.lower2_h = ParamSliderWidget("H min", 0, 180, 170)
        self.lower2_s = ParamSliderWidget("S min", 0, 255, 100)
        self.lower2_v = ParamSliderWidget("V min", 0, 255, 100)
        self.upper2_h = ParamSliderWidget("H max", 0, 180, 180)
        self.upper2_s = ParamSliderWidget("S max", 0, 255, 255)
        self.upper2_v = ParamSliderWidget("V max", 0, 255, 255)
        
        group2_layout.addWidget(QLabel("Lower:"), 0, 0)
        group2_layout.addWidget(self.lower2_h, 0, 1)
        group2_layout.addWidget(self.lower2_s, 0, 2)
        group2_layout.addWidget(self.lower2_v, 0, 3)
        group2_layout.addWidget(QLabel("Upper:"), 1, 0)
        group2_layout.addWidget(self.upper2_h, 1, 1)
        group2_layout.addWidget(self.upper2_s, 1, 2)
        group2_layout.addWidget(self.upper2_v, 1, 3)
        
        group2.setLayout(group2_layout)
        
        # Botones
        btn_layout = QHBoxLayout()
        self.btn_reset = QPushButton("Restaurar Valores por Defecto")
        self.btn_reset.clicked.connect(self.reset_values)
        self.btn_publish = QPushButton("Publicar Parámetros")
        self.btn_publish.clicked.connect(self.publish_params)
        
        btn_layout.addWidget(self.btn_reset)
        btn_layout.addWidget(self.btn_publish)
        
        # Timer para publicación automática (cada 100ms)
        self.auto_publish = True
        self.timer = QTimer()
        self.timer.timeout.connect(self.publish_params)
        self.timer.start(100)  # 100 ms
        
        # Añadir todo al layout principal
        main_layout.addWidget(group1)
        main_layout.addWidget(group2)
        main_layout.addLayout(btn_layout)
        
        # Status bar
        self.statusBar().showMessage("Listo - Publicando parámetros automáticamente")
        
        self.ros_node.get_logger().info("GUI iniciada")
    
    def get_params_array(self):
        """Retorna lista de 12 parámetros para ROS"""
        return [
            self.lower1_h.get_value(), self.lower1_s.get_value(), self.lower1_v.get_value(),
            self.upper1_h.get_value(), self.upper1_s.get_value(), self.upper1_v.get_value(),
            self.lower2_h.get_value(), self.lower2_s.get_value(), self.lower2_v.get_value(),
            self.upper2_h.get_value(), self.upper2_s.get_value(), self.upper2_v.get_value()
        ]
    
    def reset_values(self):
        """Restaurar valores por defecto"""
        self.lower1_h.slider.setValue(0)
        self.lower1_s.slider.setValue(100)
        self.lower1_v.slider.setValue(100)
        self.upper1_h.slider.setValue(10)
        self.upper1_s.slider.setValue(255)
        self.upper1_v.slider.setValue(255)
        
        self.lower2_h.slider.setValue(170)
        self.lower2_s.slider.setValue(100)
        self.lower2_v.slider.setValue(100)
        self.upper2_h.slider.setValue(180)
        self.upper2_s.slider.setValue(255)
        self.upper2_v.slider.setValue(255)
        
        self.statusBar().showMessage("Valores restaurados a defecto", 2000)
    
    def publish_params(self):
        """Publicar parámetros actuales a ROS"""
        params = self.get_params_array()
        msg = Int32MultiArray()
        msg.data = params
        
        self.ros_node.publisher.publish(msg)
        # Mostrar solo cada 1 segundo para no saturar
        if not hasattr(self, '_last_status') or (self.timer.remainingTime() % 1000 < 100):
            self.statusBar().showMessage(f"Publicado: Hmin1={params[0]}, Hmax1={params[3]}, Hmin2={params[6]}, Hmax2={params[9]}", 500)

class ROSParamNode(Node):
    """Nodo ROS que publica parámetros desde la GUI"""
    def __init__(self):
        super().__init__('stop_sign_params_gui')
        self.publisher = self.create_publisher(Int32MultiArray, '/stop_sign/params', 10)
        self.get_logger().info("Nodo publicador iniciado")

def main(args=None):
    rclpy.init(args=args)
    
    # Inicializar nodo ROS
    ros_node = ROSParamNode()
    
    # Inicializar aplicación Qt
    app = QApplication(sys.argv)
    
    # Crear GUI con el nodo
    gui = StopSignGUI(ros_node)
    gui.show()
    
    # Timer para spin ROS en segundo plano
    def ros_spin():
        rclpy.spin_once(ros_node, timeout_sec=0)
    
    ros_timer = QTimer()
    ros_timer.timeout.connect(ros_spin)
    ros_timer.start(10)  # 10ms
    
    try:
        sys.exit(app.exec())
    except KeyboardInterrupt:
        pass
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
