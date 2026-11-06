#!/usr/bin/env python3
"""
aeb.py - Frenado automático de emergencia (AEB) por tiempo a colisión (TTC).

Cadena de comandos:
    teleop / seguidor -> twist_mux -> /cmd_vel_mux -> [AEB] -> /cmd_vel -> Gazebo

Para cada rayo i del LiDAR (ángulo θ_i medido desde el eje x del robot):
    ṙ_i   = -v · cos(θ_i)          velocidad a la que cambia la distancia r_i
    TTC_i = r_i / max(-ṙ_i, 0)     tiempo hasta el choque (inf si el punto no se acerca)
Si min(TTC_i) < umbral(v) se publica velocidad cero (parada total).

Justificación del umbral:
    Para no chocar, la distancia libre d debe cubrir lo recorrido durante la latencia
    (v·t_lat) más la distancia de frenado (v²/(2a)):   d > v·t_lat + v²/(2a)
    Dividiendo entre v (TTC = d/v):                   TTC > t_lat + v/(2a)
    => umbral(v) = t_latencia + |v|/(2·a_frenado) + t_margen
    Ejemplo con v = 1.0 m/s (máximo del plugin): 0.15 + 0.25 + 0.10 = 0.50 s

Contra los falsos frenados (curvas, estrechamiento):
    1) cos(θ_i): un rayo lateral (θ = ±90°) no se acerca -> TTC = inf.
    2) Pasillo: solo cuentan los puntos que el robot barrería yendo recto
       (|r·sinθ| < semiancho). Las paredes paralelas quedan fuera.
    3) Contorno: r_i se mide desde el LiDAR (centro del robot); se le resta la
       distancia del centro al borde del chasis en la dirección del rayo.
"""

import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32


class AEB(Node):

    def __init__(self):
        super().__init__('aeb')

        # ---------------- Parámetros ----------------
        # Geometría del robot vista desde el LiDAR (centro de base_link)
        self.declare_parameter('semilargo', 0.10)    # m: chasis 0.1986 / 2
        self.declare_parameter('semiancho', 0.077)   # m: rueda en y = 0.067 + 0.019 / 2
        self.declare_parameter('margen_dist', 0.02)  # m: colchón extra delante y detrás
        # Umbral dinámico
        self.declare_parameter('t_latencia', 0.15)   # s: scan a 10 Hz (0.10) + bucle a 20 Hz (0.05)
        self.declare_parameter('a_frenado', 2.0)     # m/s²: aceleración máxima del plugin DiffDrive
        self.declare_parameter('t_margen', 0.10)     # s: margen de seguridad
        # Funcionamiento
        self.declare_parameter('timeout_cmd', 0.6)   # s: un comando más viejo se toma como cero
        self.declare_parameter('frecuencia', 20.0)   # Hz: publicación en /cmd_vel

        gp = self.get_parameter
        self.semilargo = gp('semilargo').value + gp('margen_dist').value
        self.semiancho = gp('semiancho').value
        self.t_latencia = gp('t_latencia').value
        self.a_frenado = gp('a_frenado').value
        self.t_margen = gp('t_margen').value
        self.timeout_cmd = gp('timeout_cmd').value

        # ---------------- Estado ----------------
        self.n_rayos = 0         # 0 = todavía no ha llegado ningún scan
        self.mascara = None      # rayos válidos y dentro del pasillo
        self.r = None            # distancias medidas (inf si el rayo no es válido)
        self.r_libre = None      # distancias menos el contorno del robot
        self.v_odom = 0.0
        self.ultimo_cmd = Twist()
        self.t_cmd = None
        self.frenando = False

        # ---------------- Comunicaciones ----------------
        self.create_subscription(LaserScan, 'scan', self.cb_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, 'odom', self.cb_odom, 10)
        self.create_subscription(Twist, 'cmd_vel_mux', self.cb_cmd, 10)
        self.pub_cmd = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub_ttc = self.create_publisher(Float32, 'aeb/ttc_min', 10)
        self.pub_activo = self.create_publisher(Bool, 'aeb/activo', 10)

        self.create_timer(1.0 / gp('frecuencia').value, self.bucle)
        self.get_logger().info(
            f'AEB listo: umbral(v) = {self.t_latencia} + |v|/{2 * self.a_frenado} + '
            f'{self.t_margen} s  (con v = 1 m/s: {self.umbral(1.0):.2f} s)')

    # ------------------------------------------------------------------
    def preparar_geometria(self, msg):
        """Se ejecuta con el primer scan: ángulo, cos, sin y contorno para cada rayo."""
        self.n_rayos = len(msg.ranges)
        self.ang = msg.angle_min + np.arange(self.n_rayos) * msg.angle_increment
        self.cos_a = np.cos(self.ang)
        self.sin_a = np.sin(self.ang)
        # Distancia del centro al borde del rectángulo del robot siguiendo cada rayo:
        # el rayo sale por delante/detrás (semilargo/|cos|) o por el lado (semiancho/|sin|).
        with np.errstate(divide='ignore'):
            por_frente = self.semilargo / np.abs(self.cos_a)
            por_lado = self.semiancho / np.abs(self.sin_a)
        self.d_cuerpo = np.minimum(por_frente, por_lado)
        self.get_logger().info(
            f'LiDAR: {self.n_rayos} rayos, de {math.degrees(self.ang[0]):.1f}° '
            f'a {math.degrees(self.ang[-1]):.1f}°')

    def cb_scan(self, msg):
        primer_scan = len(msg.ranges) != self.n_rayos
        if primer_scan:
            self.preparar_geometria(msg)

        r = np.asarray(msg.ranges, dtype=float)
        # Rayos válidos: se descartan inf/NaN y lecturas fuera del rango del sensor
        with np.errstate(invalid='ignore'):
            validos = np.isfinite(r) & (r >= msg.range_min) & (r <= msg.range_max)
        self.r = np.where(validos, r, np.inf)

        # Pasillo: coordenada lateral del punto |y| = |r·sinθ| menor que el semiancho
        y = np.where(validos, r, 0.0) * self.sin_a
        self.mascara = validos & (np.abs(y) < self.semiancho)

        # Distancia libre entre el borde del robot y el obstáculo
        self.r_libre = np.maximum(self.r - self.d_cuerpo, 0.0)

        if primer_scan:
            dentro = int(np.sum(self.r < self.d_cuerpo))
            if dentro > 0:
                self.get_logger().warn(
                    f'{dentro} rayos caen dentro del contorno del robot: '
                    '¿el LiDAR se ve a sí mismo o el robot nació pegado a una pared?')

    def cb_odom(self, msg):
        self.v_odom = msg.twist.twist.linear.x   # velocidad medida (marco base_link)

    def cb_cmd(self, msg):
        self.ultimo_cmd = msg
        self.t_cmd = self.get_clock().now()

    # ------------------------------------------------------------------
    def umbral(self, v):
        """Tiempo de reacción + tiempo equivalente de frenado + margen (ver cabecera)."""
        return self.t_latencia + abs(v) / (2.0 * self.a_frenado) + self.t_margen

    def ttc_minimo(self, v):
        """Devuelve (TTC mínimo en s, índice del rayo crítico) para una velocidad v con signo."""
        if self.mascara is None or abs(v) < 1e-3:
            return math.inf, -1
        r_punto = -v * self.cos_a                     # ṙ_i = -v·cos(θ_i)
        acercamiento = np.maximum(-r_punto, 0.0)      # max(-ṙ_i, 0)
        cuentan = self.mascara & (acercamiento > 0.0)
        if not np.any(cuentan):
            return math.inf, -1
        ttc = np.full(self.n_rayos, math.inf)
        ttc[cuentan] = self.r_libre[cuentan] / acercamiento[cuentan]   # TTC_i
        i = int(np.argmin(ttc))
        return float(ttc[i]), i

    def bucle(self):
        # 1) Comando de entrada con "hombre muerto": si es viejo, se toma como cero
        cmd = self.ultimo_cmd
        if self.t_cmd is None or \
                (self.get_clock().now() - self.t_cmd).nanoseconds * 1e-9 > self.timeout_cmd:
            cmd = Twist()
        v_cmd = cmd.linear.x
        v_odom = self.v_odom

        # 2) TTC con la velocidad medida (fórmula del enunciado) y con la pedida (anticipación)
        ttc_odom, i_odom = self.ttc_minimo(v_odom)
        ttc_cmd, i_cmd = self.ttc_minimo(v_cmd)
        peligro_odom = ttc_odom < self.umbral(v_odom)
        peligro_cmd = ttc_cmd < self.umbral(v_cmd)

        # 3) Decisión: se deja pasar un comando que se aleja (p. ej. marcha atrás para salir)
        se_aleja = v_cmd * v_odom < 0.0
        frenar = peligro_cmd or (peligro_odom and not se_aleja)

        self.pub_cmd.publish(Twist() if frenar else cmd)
        self.pub_ttc.publish(Float32(data=min(ttc_odom, 10.0)))
        self.pub_activo.publish(Bool(data=frenar))

        # 4) Mensajes solo cuando cambia el estado (para no inundar la terminal)
        if frenar and not self.frenando:
            ttc, i, v = (ttc_cmd, i_cmd, v_cmd) if peligro_cmd else (ttc_odom, i_odom, v_odom)
            self.get_logger().warn(
                f'¡AEB ACTIVADO! TTC = {ttc:.2f} s < umbral = {self.umbral(v):.2f} s | '
                f'rayo a {math.degrees(self.ang[i]):+.0f}°, r = {self.r[i]:.2f} m, '
                f'v = {v:+.2f} m/s')
        elif not frenar and self.frenando:
            self.get_logger().info('AEB liberado')
        self.frenando = frenar


def main():
    rclpy.init()
    nodo = AEB()
    try:
        rclpy.spin(nodo)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        nodo.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
