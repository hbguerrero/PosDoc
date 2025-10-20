#!/usr/bin/env python3
# wall_follower_steer.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import os, csv
from datetime import datetime

class WallFollowerSteer(Node):
    def __init__(self):
        super().__init__('wall_follower_steer_fuzzy')

        #Parametros CSV
        self.declare_parameter('csv_dir', os.path.expanduser('~/CSV_ROBOT'))
        self.declare_parameter('ErrorDist_base', 'csv_ErrorDist') 
        
        csv_dir      = self.get_parameter('csv_dir').value
        ErrorDist_base     = self.get_parameter('ErrorDist_base').value

        os.makedirs(csv_dir, exist_ok=True)
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        
        # --- ERROR DE DISTANCIA CSV ---
        self.ErrorDist_path = os.path.join(csv_dir, f'{ErrorDist_base}_{ts}.csv')
        self.csv_ErrorDist = open(self.ErrorDist_path, 'w', newline='', buffering=1)
        self.writer_ErrorDist = csv.writer(self.csv_ErrorDist)
        self.writer_ErrorDist.writerow(['ros_time_s', 'Error_Distancia'])
        self.get_logger().info(f'Error_Distancia → {self.ErrorDist_path}')
 

        # ---------- Parámetros ----------
        self.declare_parameter('in_topic', 'lidarLeft')           # distancia al muro (m)
        self.declare_parameter('out_topic', 'velocidad_deseada')  # Ángulo deseado (deg) → ESP32SerialBridge
        self.declare_parameter('setpoint', 0.50)                  # m
        self.declare_parameter('angle_limit_deg', 45.0)           # saturación de salida (±deg)
        self.declare_parameter('dtheta_max_deg', 20.0)            # límite de cambio por ciclo (anti tirones)

        self.in_topic  = self.get_parameter('in_topic').value
        self.out_topic = self.get_parameter('out_topic').value
        self.setpoint  = float(self.get_parameter('setpoint').value)
        self.AMAX      = float(self.get_parameter('angle_limit_deg').value)   # ±45
        self.DTHMAX    = float(self.get_parameter('dtheta_max_deg').value)    # deg/ciclo

        
        # Subscripción / Publicación
        self.sub = self.create_subscription(Float32, self.in_topic, self.cb_lidar, 10)
        self.pub = self.create_publisher(Float32, self.out_topic, 10)

        # Control difuso
        self._build_fuzzy()

        # Estado
        self.steer_prev = 0.0

        self.get_logger().info(
            f'WallFollowerSteer listo | in:{self.in_topic} → out:{self.out_topic} | '
            f'setpoint={self.setpoint:.2f} m, sat=±{self.AMAX:.1f}°, dθ_max={self.DTHMAX:.1f}°/ciclo\n'
        )

    # ---------- Definición del controlador difuso ----------
    def _build_fuzzy(self):
        # error = setpoint - distancia (m):  negativo = lejos; positivo = muy cerca
        self.err = ctrl.Antecedent(np.arange(-1.0, 1.001, 0.001), 'error')

        # salida: ángulo dirección (deg). CONVENCIÓN: derecha (+), izquierda (−)
        self.delta = ctrl.Consequent(np.arange(-45.0, 45.01, 0.01), 'delta')

        # Pertenencias del error
        self.err['NL'] = fuzz.trimf(self.err.universe, [-1.00, -0.10, -0.02])  # lejos del muro
        self.err['NS'] = fuzz.trimf(self.err.universe, [-0.06, -0.02,  0.00])
        self.err['Z']  = fuzz.trimf(self.err.universe, [-0.01,  0.00,  0.01])
        self.err['PS'] = fuzz.trimf(self.err.universe, [ 0.00,  0.02,  0.06])
        self.err['PL'] = fuzz.trimf(self.err.universe, [ 0.02,  0.10,  1.00])  # muy cerca

        # Pertenencias de salida (derecha +, izquierda −)
        self.delta['LL'] = fuzz.trimf(self.delta.universe, [-45, -45, -20])  # Left  Large  (−)
        self.delta['LS'] = fuzz.trimf(self.delta.universe, [-25, -10,   0])  # Left  Small  (−)
        self.delta['Z']  = fuzz.trimf(self.delta.universe, [  -3,   0,   3])  # Casi recto
        self.delta['RS'] = fuzz.trimf(self.delta.universe, [   0,  10,  25])  # Right Small (+)
        self.delta['RL'] = fuzz.trimf(self.delta.universe, [  20,  45,  45])  # Right Large (+)

        # Reglas (muro a la IZQUIERDA):
        # error < 0 (lejos)  → girar DERECHA (+) para acercarse
        # error > 0 (cerca)  → girar IZQUIERDA (−) para alejarse
        rules = [
            ctrl.Rule(self.err['NL'], consequent=self.delta['LL']),
            ctrl.Rule(self.err['NS'], consequent=self.delta['LS']),
            ctrl.Rule(self.err['Z'],  consequent=self.delta['Z']),
            ctrl.Rule(self.err['PS'], consequent=self.delta['RS']),
            ctrl.Rule(self.err['PL'], consequent=self.delta['RL']),
        ]

        system = ctrl.ControlSystem(rules)
        self.sim = ctrl.ControlSystemSimulation(system)

    # ---------- Callback ----------
    def cb_lidar(self, msg: Float32):
        dist = float(msg.data)
        e = self.setpoint - dist

        # Fuzzy
        self.sim.input['error'] = e
        self.sim.compute()
        steer = float(self.sim.output['delta'])  # grados (der +, izq −)

        # Saturación ±AMAX
        steer = float(np.clip(steer, -self.AMAX, self.AMAX))

        # Limitador de tasa
        dtheta = steer - self.steer_prev
        if dtheta >  self.DTHMAX: steer = self.steer_prev + self.DTHMAX
        if dtheta < -self.DTHMAX: steer = self.steer_prev - self.DTHMAX
        self.steer_prev = steer

        # Publicar como Float32 (grados)
        self.pub.publish(Float32(data=steer))

        # Log y CSV
        self.get_logger().info(f"steer:{steer:.2f} deg")
        if self.csv_ErrorDist and not self.csv_ErrorDist.closed:
            t = self.get_clock().now().nanoseconds / 1e9
            self.writer_ErrorDist.writerow([f'{t:.9f}', f'{float(e):.6f}'])

        

    def destroy_node(self):
        super().destroy_node()

def main():
    rclpy.init()
    node = WallFollowerSteer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
