#!/usr/bin/env python3
import rclpy, math
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import os, csv
import sys
from rplidar import RPLidar, RPLidarException
from datetime import datetime


qos_profile = QoSProfile(depth=10)
qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT

def wrap_to_pi(x: float) -> float:
    """Normaliza a (-π, π]."""
    return math.atan2(math.sin(x), math.cos(x))

class LidarLeftWithIMU(Node):
    def __init__(self):
        super().__init__('lidar_left_and_imu')
        self.lidar = RPLidar('/dev/ttyUSB1', baudrate=115200)
        self.eYaw = 0.0
        self.conic_aperture = math.radians(15)
        self.max_range = 5.0
        self.max_proj = 5.0
        self.use_projection = True

        # Parámetros CSV existentes
        self.declare_parameter('csv_dir', os.path.expanduser('~/CSV_ROBOT'))
        self.declare_parameter('LidarLeft_base', 'csv_LidarLeft')
        self.declare_parameter('ImuError_base', 'csv_ImuError')
        self.declare_parameter('LecValVerde_base', 'csv_LecValVerde')
        self.declare_parameter('LecValRojo_base', 'csv_LecValRojo')

        self.frame_id = 'laser_frame'
        self.angle_min = -math.pi
        self.angle_max = math.pi
        self.range_min = 0.05
        self.range_max = 12.0
        self.scan_time = 0.05  # ~10-15 Hz

        csv_dir       = self.get_parameter('csv_dir').value
        LidarLeft_base = self.get_parameter('LidarLeft_base').value
        ImuError_base  = self.get_parameter('ImuError_base').value
        LecValVerde_base    = self.get_parameter('LecValVerde_base').value
        LecValRojo_base    = self.get_parameter('LecValRojo_base').value


        os.makedirs(csv_dir, exist_ok=True)
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')

        # --- CSV DISTANCIA (igual que tenías) ---
        self.LidarLeft_path = os.path.join(csv_dir, f'{LidarLeft_base}_{ts}.csv')
        self.csv_LidarLeft = open(self.LidarLeft_path, 'w', newline='', buffering=1)
        self.writer_LidarLeft = csv.writer(self.csv_LidarLeft)
        self.writer_LidarLeft.writerow(['ros_time_s', 'Lidar_Left'])
        self.get_logger().info(f'LidarLeft → {self.LidarLeft_path}')

        # --- CSV IMU_ERROR_YAW (igual) ---
        self.ImuError_path = os.path.join(csv_dir, f'{ImuError_base}_{ts}.csv')
        self.csv_ImuError = open(self.ImuError_path, 'w', newline='', buffering=1)
        self.writer_ImuError = csv.writer(self.csv_ImuError)
        self.writer_ImuError.writerow(['ros_time_s', 'Imu_Error_yaw'])
        self.get_logger().info(f'Imu_Error_yaw → {self.ImuError_path}')

        # --- CSV LECTURAS VÁLIDAS (cono verde) (igual) ---
        self.LecValVerde_path = os.path.join(csv_dir, f'{LecValVerde_base}_{ts}.csv')
        self.csv_LecValVerde = open(self.LecValVerde_path, 'w', newline='', buffering=1)
        self.writer_LecValVerde = csv.writer(self.csv_LecValVerde)
        self.writer_LecValVerde.writerow(['ros_time_s', 'Lecturas_ValidasVerde'])
        self.get_logger().info(f'Lecturas_ValidasVerde → {self.LecValVerde_path}')
        
         # --- CSV LECTURAS VÁLIDAS (cono rojo) (igual) ---
        self.LecValRojo_path = os.path.join(csv_dir, f'{LecValRojo_base}_{ts}.csv')
        self.csv_LecValRojo = open(self.LecValRojo_path, 'w', newline='', buffering=1)
        self.writer_LecValRojo = csv.writer(self.csv_LecValRojo)
        self.writer_LecValRojo.writerow(['ros_time_s', 'Lecturas_ValidasRojo'])
        self.get_logger().info(f'Lecturas_ValidasRojo → {self.LecValRojo_path}')
        
        

        # ── Parámetros configurables (verdes, igual) ───────────────────────────
        self.declare_parameter('aperture_deg', 100.0)   # ancho total del cono (grados)
        self.declare_parameter('max_range', 8.0)        # m
        self.declare_parameter('use_projection', True)  # proyección escalar
        self.declare_parameter('max_proj', 0.7)         # m (umbral proyección verde)
        # ───────────────────────────────────────────────────────────────────────

        # === NUEVO: parámetros del CONO ESTRECHO (rojo) ===
        self.declare_parameter('narrow_aperture_deg', 6.0)   # ancho total (deg)
        self.declare_parameter('narrow_max_proj', 0.7)      # m (umbral proyección estrecho)
        # === NUEVO: parámetros de CONTEO NARROW ===
        self.declare_parameter('narrow_min_valids', 2)       # mínimo de lecturas válidas para contar
        self.declare_parameter('narrow_refractory_sec', 2.0) # tiempo mínimo entre conteos

        # IMU
        self.initial_imu_angle = None
        self.imu_angle = 0.0
        self.eYaw = 0.0

        # Publishers existentes (no añadimos nuevos)
        self.pub_left = self.create_publisher(Float32, 'lidarLeft', 10)
        self.pub_scan = self.create_publisher(LaserScan, 'scan', qos_profile)
        self.pub_error_yaw = self.create_publisher(Float32, 'imu_error_yaw', 10)

        # Subscriber IMU
        self.sub_imu = self.create_subscription(
            Float32, 'imu_heading', self.callback_imu, qos_profile_sensor_data
        )

        # Timer
        self.create_timer(0.10, self.process_scan)

        # Cache de parámetros
        self._update_params()

        # === Estado CONTEO NARROW ===
        self.narrow_count_total = 0
        self.last_narrow_event_t = -1.0

        # Watchdog
        self.last_imu_msg = self.get_clock().now()
        self.create_timer(1.0, self._watchdog)

    def _update_params(self):
        ap_deg = float(self.get_parameter('aperture_deg').value)
        self.conic_aperture = math.radians(ap_deg) / 2.0
        self.max_range = float(self.get_parameter('max_range').value)
        self.use_projection = bool(self.get_parameter('use_projection').value)
        self.max_proj = float(self.get_parameter('max_proj').value)

        # cache CONO ESTRECHO
        nar_deg = float(self.get_parameter('narrow_aperture_deg').value)
        self.narrow_aperture = math.radians(nar_deg) / 2.0
        self.narrow_max_proj = float(self.get_parameter('narrow_max_proj').value)

        # parámetros de conteo
        self.narrow_min_valids = int(self.get_parameter('narrow_min_valids').value)
        self.narrow_refractory = float(self.get_parameter('narrow_refractory_sec').value)

    def callback_imu(self, msg: Float32):
        current_rad = math.radians(float(msg.data))
        current_rad = wrap_to_pi(current_rad)

        if self.initial_imu_angle is None:
            self.initial_imu_angle = current_rad
            self.get_logger().info(f"📌 IMU referencia inicial: {self.initial_imu_angle:.3f} rad")

        self.imu_angle = current_rad
        self.eYaw = wrap_to_pi(self.initial_imu_angle - self.imu_angle)

        self.pub_error_yaw.publish(Float32(data=self.eYaw))
        self.last_imu_msg = self.get_clock().now()

    def wrap_to_pi(angle):
        """Normaliza ángulo a (-π, π]"""
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def _now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def process_scan(self):
        try:
            scan_data = []
            for i, scan in enumerate(self.lidar.iter_scans(max_buf_meas=500, min_len=5)):
                scan_data = scan
                break  # Solo un ciclo

            if not scan_data:
                self.get_logger().warn("LiDAR scan vacío o fallido")
                return

            # Centro del cono para la IZQUIERDA del robot:
            conic_center = wrap_to_pi(-math.pi/2 + self.eYaw)

            # Acumuladores cono verde (igual que antes)
            sum_val, count = 0.0, 0
            # Acumuladores cono estrecho (nuevo)
            sum_val_nar, count_nar = 0.0, 0

            for quality, angle_deg, distance_mm in scan_data:
                a = math.radians(angle_deg)
                r = distance_mm / 1000.0

                if a > math.pi:
                    a -= 2.0 * math.pi

                # --- CONO VERDE (igual) ---
                if (conic_center - self.conic_aperture) <= a <= (conic_center + self.conic_aperture):
                    if 0.0 < r <= self.max_range:
                        if self.use_projection:
                            angle_diff = a - conic_center
                            proj = r * math.cos(angle_diff)
                            if 0.0 < proj <= self.max_proj:
                                sum_val += proj
                                count += 1
                        else:
                            sum_val += r
                            count += 1

                # --- CONO ESTRECHO (nuevo, solo para conteo/log) ---
                if (conic_center - self.narrow_aperture) <= a <= (conic_center + self.narrow_aperture):
                    if 0.0 < r <= self.max_range:
                        angle_diff_n = a - conic_center
                        proj_n = r * math.cos(angle_diff_n)
                        if 0.0 < proj_n <= self.narrow_max_proj:
                            sum_val_nar += proj_n
                            count_nar += 1

            # --- Publicación CONO VERDE (igual) ---
            if count > 0:
                avg = sum_val / count
                self.pub_left.publish(Float32(data=avg))
                self.get_logger().info(
                    f"📏 Dist. lateral: {avg:.3f} m (validas={count}, centro={conic_center:.2f} rad, eYaw={self.eYaw:.6f})"
                )
                t = self._now_s()
                if self.csv_LidarLeft and not self.csv_LidarLeft.closed:
                    self.writer_LidarLeft.writerow([f'{t:.9f}', f'{float(avg):.6f}'])
                if self.csv_ImuError and not self.csv_ImuError.closed:
                    self.writer_ImuError.writerow([f'{t:.9f}', f'{float(self.eYaw):.6f}'])
                if self.csv_LecValVerde and not self.csv_LecValVerde.closed:
                    self.writer_LecValVerde.writerow([f'{t:.9f}', f'{float(count):.6f}'])
            else:
                self.get_logger().warn("No se encontraron lecturas válidas en el cono.")

            # --- SOLO LOG + CONTEO: lecturas válidas del CONO ESTRECHO ---
            t = self._now_s()
            if count_nar > 0:
                avg_nar = sum_val_nar / count_nar
                self.get_logger().info(f"🔴 NARROW válidas={count_nar} | avg={avg_nar:.3f} m, Postes={self.narrow_count_total}")
            else:
                self.get_logger().info(f"🔴 NARROW válidas=0, Postes={self.narrow_count_total}")

            # === CONTEO NARROW con refractario (nuevo) ===
            if count_nar >= self.narrow_min_valids:
                if (self.last_narrow_event_t < 0.0) or ((t - self.last_narrow_event_t) >= self.narrow_refractory):
                    self.narrow_count_total += 1
                    self.last_narrow_event_t = t
                    self.get_logger().info(f"✅ CONTEO NARROW +1 → Postes={self.narrow_count_total} (t={t:.2f}s)")
                else:
                    # Dentro del refractario: no contar, solo notificar (opcional)
                    remaining = self.narrow_refractory - (t - self.last_narrow_event_t)
                    self.get_logger().debug(f"⏳ En refractario ({remaining:.2f}s restantes), no se cuenta.")
            t = self._now_s()
            if self.csv_LecValRojo and not self.csv_LecValRojo.closed:
                    self.writer_LecValRojo.writerow([f'{t:.9f}', f'{float(count_nar):.6f}'])

            # --- Publicar LaserScan (igual) ---
            for scan in self.lidar.iter_scans(scan_type='express'):
                if not scan or len(scan) == 0:
                    self.get_logger().warn("Scan vacío.")
                    continue

                msg = LaserScan()
                now = self.get_clock().now().to_msg()
                msg.header.stamp = now
                msg.header.frame_id = "laser_frame"

                angles = [math.radians(pt[1]) for pt in scan]
                ranges = [pt[2] / 1000.0 for pt in scan]

                msg.angle_min = min(angles)
                msg.angle_max = max(angles)
                msg.angle_increment = (msg.angle_max - msg.angle_min) / len(angles)
                msg.time_increment = 0.0
                msg.scan_time = 0.05
                msg.range_min = 0.05
                msg.range_max = 12.0
                msg.ranges = ranges

                self.pub_scan.publish(msg)
                break

        except Exception as e:
            self.get_logger().error(f"Error procesando scan: {e}")
            self.get_logger().error('[+] Deteniendo el LIDAR correctamente...')
            self.lidar.stop()
            self.lidar.stop_motor()
            self.lidar.disconnect()
            self.get_logger().error('[✓] LIDAR apagado y desconectado.')
        except KeyboardInterrupt:
            self.get_logger().error('\n[!] Interrumpido por el usuario (Ctrl+C)')
            self.get_logger().error('[+] Deteniendo el LIDAR correctamente...')
            self.lidar.stop()
            self.lidar.stop_motor()
            self.lidar.disconnect()
            self.get_logger().error('[✓] LIDAR apagado y desconectado.')
        
    def _watchdog(self):
        if (self.get_clock().now() - self.last_imu_msg).nanoseconds > 2e9:
            self.get_logger().warn("⏱️ No llegan mensajes en /imu_heading desde hace >2 s.")

    def destroy_node(self):
        try:
            self.lidar.stop()
            self.lidar.stop_motor()
            self.lidar.disconnect()
            if self.csv_LidarLeft and not self.csv_LidarLeft.closed:
                self.csv_LidarLeft.flush(); self.csv_LidarLeft.close()
            if self.csv_ImuError and not self.csv_ImuError.closed:
                self.csv_ImuError.flush(); self.csv_ImuError.close()
            if self.csv_LecValVerde and not self.csv_LecValVerde.closed:
                self.csv_LecValVerde.flush(); self.csv_LecValVerde.close()
            if self.csv_LecValRojo and not self.csv_LecValRojo.closed:
                self.csv_LecValRojo.flush(); self.csv_LecValRojo.close()
        except Exception as e:
            self.get_logger().warn(f'Error al cerrar archivos: {e}')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LidarLeftWithIMU()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
