# Inicializar ROS2 y crear nodos
import rclpy
from rclpy.node import Node

# Operaciones numericas
import numpy as np
from math import hypot

# Contiene la lectura del LIDAR
from sensor_msgs.msg import LaserScan

# Mover el carro 
from ackermann_msgs.msg import AckermannDriveStamped

# Odom (para posición/pose)
from nav_msgs.msg import Odometry

# Mensajes para publicar info de vueltas/tiempos (texto simple)
from std_msgs.msg import String


class ReactiveFollowGap(Node):
    def __init__(self):
    
    	# Creamos nodo "reactive_follow_gap"
        super().__init__("reactive_follow_gap")

	# Crear una suscripcion al topic /scan
        self.subscription = self.create_subscription(
            LaserScan, "/scan", self.lidar_callback, 10
        )
        
        # Crear suscripcion a /odom para conteo de vueltas y cronometro
        self.odom_sub = self.create_subscription(
            Odometry, "/ego_racecar/odom", self.odom_callback, 10
        )
        
        # Crear un publicador en topic /drive
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, "/drive", 10
        )
        
        # Publicador para información de vueltas/tiempos 
        self.lap_pub = self.create_publisher(String, "/lap_info", 10)


# --- Parámetros ajustados para Budapest ---
        
        # Zona que se anula al rededor del obs mas cercano
        self.bubble_radius_m = 1.32
        
        # tamaño de la ventana para el filtro del lidar            
        self.smoothing_kernel = 3   
        
        # Valor maximo (util) del lidar             
        self.max_range = 3.5
        
        # MUltiplicador sobre el angulo calculado                 
        self.steering_gain = 1.17
        
        # Tope maximo (rad) que permites para la direccion (giro)           
        self.max_steering = 0.15


# --- Parámetros para conteo de vueltas / cronómetro ---

        # Radios para la "zona de inicio/finish": (m)
        self.entry_radius = 1.0   # al entrar dentro -> posible conteo
        self.exit_radius = 1.5    # debe primero salir del área para armar el siguiente conteo

        # Tiempo mínimo entre vueltas (segundos) para evitar múltiples cuentas consecutivas
        self.min_lap_time = 5.0

        # Estado para detección de paso por linea de meta
        self.start_pos = None            # (x,y) en el primer odom recibido -> línea de meta centrada aquí
        self.has_left_start_zone = False # para evitar contar inmediatamente al comenzar
        self.lap_start_time = None       # timestamp (s) de inicio de la vuelta en curso
        self.last_lap_time = None        # tiempo de la última vuelta
        self.lap_times = []              # lista de tiempos por vuelta
        self.lap_count = 0
        
        self.best_lap_time = None # Inicializar el mejor tiempo como None o float('inf')
        self.MAX_LAPS = 10        # Definir el límite de vueltas

        # Flag para saber si recibimos odom
        self.odom_received = False

        # Mensaje de inicio
        self.get_logger().info("ReactiveFollowGap node started. Waiting for /odom and /scan...")


# --- Suavizar LiDAR ---
    def preprocess(self, ranges):
    	
         # Convertir a Array
         r = np.array(ranges)

	 # Corregir fallos de lectura 
	 # Rango max para que el filtro no los arrastre a 0 
         r[r == 0] = self.max_range

	 # Limita los valores al rango max del LIDAR definido 
         r[r > self.max_range] = self.max_range

	 # Array de tamaño kernell=3 de "unos"/kernell
         kernel = np.ones(self.smoothing_kernel) / self.smoothing_kernel

	 # Cada dato se convierte en el promedio de si mismo y sus vecinos 
         return np.convolve(r, kernel, mode="same") 


# --- Encontrar el gap más grande ---
    def find_largest_gap(self, free_space):
    
    	# Lista 
        gaps = []
        start = None
        
        # Recorre array de espacio libre (1=indice) (v=distancia)
        for i, v in enumerate(free_space):
        
            # Si hay espacio libre 
            if v > 0 and start is None:
            	# comienza el gap
                start = i
                
            # Si hay obstaculo 
            elif v == 0 and start is not None:
                gaps.append((start, i - 1))   #crea el gap como tupla
                start = None  		      #resetea el start
        
        # Por si termina con un gap abierto
        if start is not None:
            gaps.append((start, len(free_space)-1))

	# Devuelve gap mas grande: Restando el tamaño del gap
        return max(gaps, key=lambda g: g[1]-g[0])


# --- Mejor punto dentro del gap ---
    def select_best_point(self, gap, ranges):
        start, end = gap	#indices de inicio y fin
        
        #Indice global del array Ranges(LIdar) que tiene el valor MAX
        return start + np.argmax(ranges[start:end+1])
    
    
    
# --- Control de velocidad basado en distancia libre ---
    def compute_adaptive_speed(self, free_dist):

        # Tramos de distancia (ajústalos según Budapest)
        d1 = 1.2
        d2 = 2.25
        d3 = 3.4

        # Velocidades para cada tramo
        Vmin = 1.0
        Vm1 = 2.0
        Vm2 = 2.3
        Vmax = 2.5

        # Tramo 1 — distancia pequeña → muy lento
        if free_dist <= d1:
            return Vmin

        # Tramo 2 — crecimiento suave
        elif d1 < free_dist <= d2:
            return Vmin + (Vm1 - Vmin) * (free_dist - d1) / (d2 - d1)

        # Tramo 3 — crecimiento más rápido
        elif d2 < free_dist <= d3:
            return Vm1 + (Vm2 - Vm1) * (free_dist - d2) / (d3 - d2)

        # Tramo final — velocidad constante
        else:
            return Vmax

    
    


# --- Callback principal FTG ---
    def lidar_callback(self, msg):
        ranges = self.preprocess(msg.ranges)

        # 1. Obstáculo más cercano
        closest = np.argmin(ranges)

        # 2. Aplicar bubble (en índices)
        bubble_radius_idx = int(self.bubble_radius_m / msg.angle_increment)
        start = max(0, closest - bubble_radius_idx)
        end = min(len(ranges) - 1, closest + bubble_radius_idx)

	# Copia los valores suavizados 
        free_space = np.copy(ranges)
        free_space[start:end+1] = 0

        # 3. Encontrar gap más grande
        largest_gap = self.find_largest_gap(free_space)

        # 4. Elegir punto objetivo
        best_point = self.select_best_point(largest_gap, ranges)
        
        
        # --- Distancia libre hacia el punto objetivo ---
        free_dist = ranges[best_point]
        adaptive_speed = self.compute_adaptive_speed(free_dist)


        # 5. Convertir a ángulo
        angle = msg.angle_min + best_point* msg.angle_increment
        angle = np.clip(angle, -self.max_steering, self.max_steering)
        

        # Ajustar velocidad con base en distancia libre
        speed = adaptive_speed

        # 6. Publicar movimiento
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = float(angle)
        drive_msg.drive.speed = float(speed)
        self.drive_pub.publish(drive_msg) 
        
        


# También podemos emitir el estado de vueltas periódicamente
        if self.odom_received:
            # publicar un resumen corto para visualización en video
            lap_summary = f"laps={self.lap_count} last_lap={self.last_lap_time if self.last_lap_time is not None else '-'}"
            msg = String()
            msg.data = lap_summary
            self.lap_pub.publish(msg)
        else:
            pass
            # advertencia no invasiva (solo la primera vez)
            #self.get_logger().warn("No /odom received yet — lap counter inactive.")


# --- Callback de Odometry — usado para conteo de vueltas ---
    def odom_callback(self, odom_msg):
        # extraer posición x,y del odom
        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y

        now_s = self.get_clock().now().nanoseconds / 1e9

        # Primera vez: establecer la posición de inicio (línea meta)
        if self.start_pos is None:
            self.start_pos = (x, y)
            self.lap_start_time = now_s
            self.odom_received = True
            self.get_logger().info(f"Start position set at ({x:.2f}, {y:.2f}). Lap counter armed.")
            return

        self.odom_received = True

        # distancia al punto de inicio
        dist_to_start = hypot(x - self.start_pos[0], y - self.start_pos[1])

        # Estado máquina:
        #  - Si estamos fuera de la zona de inicio (dist > exit_radius) marcamos has_left_start_zone = True
        #  - Si antes habíamos salido y ahora entramos dentro entry_radius y además lap time > min_lap_time => contar nueva vuelta
        if not self.has_left_start_zone:
            # Si nos alejamos lo suficiente, armamos la próxima vuelta
            if dist_to_start > self.exit_radius:
                self.has_left_start_zone = True
                self.get_logger().debug("Vehicle left start zone — ready to detect lap entry.")
        else:
            # Estamos en 'armed' para contar cuando vuelva a entrar
            if dist_to_start <= self.entry_radius:
                # calcular tiempo desde lap_start_time
                lap_time = now_s - self.lap_start_time if self.lap_start_time is not None else None

                # verificar mínimo tiempo entre vueltas
                if lap_time is None or lap_time >= self.min_lap_time:
                    # contar la vuelta
                    self.lap_count += 1
                    self.last_lap_time = lap_time
                    self.lap_times.append(lap_time if lap_time is not None else 0.0)


                    # --- NUEVA LÍNEA 1: Actualizar el Mejor Tiempo ---
                    if self.best_lap_time is None or lap_time < self.best_lap_time:
                        self.best_lap_time = lap_time
                    
                    
                    # publicar info clara 
                    info = f"VUELTA {self.lap_count} completada — time: {lap_time:.3f} s"
                    self.get_logger().info(info)
                    msg = String()
                    msg.data = info
                    self.lap_pub.publish(msg)
                    
                    
                    # --- NUEVO BLOQUE 2: Reporte final de 10 vueltas ---
                    if self.lap_count >= self.MAX_LAPS:
                        final_info = (f"COMPETENCIA FINALIZADA ({self.MAX_LAPS} vueltas). "
                                  f"🏆 Mejor tiempo: {self.best_lap_time:.3f} s")
                        self.get_logger().info("##################################################")
                        self.get_logger().info(final_info)
                        self.get_logger().info("##################################################")
                        # Opcional: Desuscribirse del odom para detener el conteo
                        # self.odom_sub.destroy()
                    # --------------------------------------------------
                    
                    

                    # reiniciar timers para la siguiente vuelta
                    self.lap_start_time = now_s
                    # armar para la siguiente vuelta: requerimos que salga de la zona otra vez
                    self.has_left_start_zone = False
                else:
                    # si la vuelta fue demasiado corta, la ignoramos (posible rebote/ruido)
                    self.get_logger().debug(f"Ignored potential lap (lap_time={lap_time:.3f}s < min {self.min_lap_time}s)")
                    # No cambiamos el estado; esperamos salir y volver a entrar

    
    
    # Función auxiliar para obtener resumen (puedes llamarla desde la sustentación)
    def get_lap_summary(self):
        return {
            "lap_count": self.lap_count,
            "lap_times": self.lap_times,
            "last_lap_time": self.last_lap_time
        }



def main(args=None):
    rclpy.init(args=args)
    node = ReactiveFollowGap()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

