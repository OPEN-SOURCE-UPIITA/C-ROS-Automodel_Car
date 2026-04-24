import socket
import threading

# Configuración del servidor (Escucha en todas las interfaces)
HOST = '0.0.0.0'  
PORT = 65432      

# Variable para guardar la conexión hacia el robot
robot_conn = None

def manejar_cliente(conn, addr):
    global robot_conn
    print(f"[NUEVA CONEXIÓN] Dirección {addr} conectada.")
    
    try:
        # El cliente debe enviar un "saludo" para identificarse
        identidad = conn.recv(1024).decode('utf-8').strip()
        
        if identidad == "SOY_EL_ROBOT":
            robot_conn = conn
            print("[INFO] Robot registrado correctamente.")
            # Mantiene viva la conexión del robot
            while True:
                if not conn.recv(1024): 
                    break
                    
        elif identidad == "SOY_EL_USUARIO":
            print("[INFO] Usuario conectado. Listo para retransmitir comandos.")
            while True:
                # Recibe el comando del usuario
                data = conn.recv(1024)
                if not data:
                    break
                
                # Si el robot está conectado, le reenvía el mensaje crudo
                if robot_conn:
                    try:
                        robot_conn.sendall(data)
                    except Exception as e:
                        print(f"[ERROR] No se pudo enviar al robot: {e}")
                else:
                    print("[ADVERTENCIA] Comando recibido, pero el robot no está conectado.")
                    
    except Exception as e:
        print(f"[ERROR] Problema con la conexión {addr}: {e}")
    finally:
        conn.close()
        if robot_conn == conn:
            robot_conn = None
            print("[INFO] El robot se ha desconectado.")

def iniciar_servidor():
    print("[INICIANDO] Servidor arrancando...")
    servidor = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    servidor.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    servidor.bind((HOST, PORT))
    servidor.listen()
    print(f"[LISTO] Servidor escuchando en el puerto {PORT}")
    
    while True:
        conn, addr = servidor.accept()
        # Crea un hilo nuevo por cada conexión (robot o usuario)
        hilo = threading.Thread(target=manejar_cliente, args=(conn, addr))
        hilo.start()

if __name__ == "__main__":
    iniciar_servidor()