#!/usr/bin/env python3
"""
seguidor_pared.py - Seguimiento de pared con PID (modo autónomo reactivo).

Publica en /cmd_vel_auto  ->  twist_mux  ->  /cmd_vel_mux  ->  AEB  ->  /cmd_vel

Geometría (para cada pared; derecha = espejo de la izquierda):
    rayo b perpendicular a la pared (±90°), rayo a a un ángulo θ de b (±(90°-θ))
    α       = atan((a·cosθ - b) / (a·sinθ))     (α < 0 si el robot va hacia esa pared)
    D_t     = b·cosα
    D_{t+1} = D_t + l·sinα                       (l = distancia de anticipación)
    e       = D_des - D_{t+1}
Con las dos paredes: D_des = (D_izq + D_der)/2 y e = D_des - D_izq (centrado).
Con una sola pared: se sigue esa pared con D_des = ancho memorizado / 2.
Control: ω = -PID(e)   (e > 0 = demasiado cerca de la izquierda -> girar a la derecha)

Sintonía (modelo cinemático, y = desvío lateral, ψ = rumbo):
    ẏ = v·ψ,  e ≈ y + l·ψ,  ω = -Kp·e   =>   ÿ + Kp·l·ẏ + Kp·v·y = 0
    ωn = sqrt(Kp·v),  ζ = Kp·l / (2·sqrt(Kp·v))
    Kp = 10, l = 0.30 m, v = 0.5 m/s  ->  ωn = 2.2 rad/s, ζ = 0.67

Velocidad: baja con |ω| (curvas) y con la distancia libre al frente,
de modo que TTC_frente ≈ 1/k_frente = 1 s > umbral del AEB (≤ 0.5 s).
"""

import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

IZQ, DER = +1, -1


class SeguidorPared(Node):

    def __init__(self):
        super().__init__('seguidor_pared')

        # Parámetros (se releen en cada scan: se pueden cambiar con 'ros2 param set')
        parametros = {
            # Geometría de los rayos
            'theta_deg': 45.0,        # ángulo entre los rayos a y b
            'lookahead': 0.42,        # m: l, crece con v para mantener xhi = 0.67 et l =0.62*2 raz de v / raz de Kp
            'd_max': 1.5,             # m: más allá, el rayo b "no ve" pared
            'alfa_max_deg': 60.0,     # |α| mayor = el rayo a entra en una abertura
            'ancho_inicial': 0.50,    # m: ancho supuesto antes de medirlo
            # PID
            'kp': 10.0,
            'ki': 0.5,
            'kd': 0.8,
            'integral_max': 0.2,      # m·s: anti-windup
            'w_max': 3.0,             # rad/s (el plugin admite 4.0)
            # Velocidad
            'v_max': 1.0,             # m/s en recta (limite del enunciado)
            'v_min': 0.08,            # m/s
            'k_curva': 0.7,           # reducción de v con |ω|/w_max
            'k_frente': 1.0,          # 1/s: v ≤ k_frente · distancia libre al frente
            'cono_frente_deg': 15.0,  # semiancho del cono frontal
            'semilargo': 0.10,        # m: del LiDAR al frente del chasis
        }
        for nombre, valor in parametros.items():
            self.declare_parameter(nombre, valor)

        # Estado del PID
        self.integral = 0.0
        self.e_ant = None
        self.t_ant = None
        self.de_filtrada = 0.0
        self.modo_ant = None
        self.ancho = None

        self.create_subscription(LaserScan, 'scan', self.cb_scan, qos_profile_sensor_data)
        self.pub_cmd = self.create_publisher(Twist, 'cmd_vel_auto', 10)
        self.pub_error = self.create_publisher(Float32, 'seguidor/error', 10)
        self.get_logger().info('Seguidor de pared listo (publica en /cmd_vel_auto)')

    def p(self, nombre):
        return self.get_parameter(nombre).value

    # ------------------------------------------------------------------
    @staticmethod
    def rango(msg, r, angulo):
        """Mediana de 5 rayos alrededor de 'angulo' (rad). inf si no hay lecturas válidas."""
        n = len(r)
        i = int(round((angulo - msg.angle_min) / msg.angle_increment))
        vecinos = r[[(i + k) % n for k in range(-2, 3)]]
        ok = np.isfinite(vecinos) & (vecinos >= msg.range_min) & (vecinos <= msg.range_max)
        return float(np.median(vecinos[ok])) if np.any(ok) else math.inf

    def distancia_pared(self, msg, r, lado):
        """Devuelve (D_{t+1}, α) para la pared de 'lado' (IZQ o DER), o None si no se ve."""
        theta = math.radians(self.p('theta_deg'))
        b = self.rango(msg, r, lado * math.pi / 2)              # rayo perpendicular
        a = self.rango(msg, r, lado * (math.pi / 2 - theta))    # rayo adelantado θ
        if not (b < self.p('d_max') and math.isfinite(a)):
            return None
        alfa = math.atan((a * math.cos(theta) - b) / (a * math.sin(theta)))
        if abs(alfa) > math.radians(self.p('alfa_max_deg')):
            return None
        d_t = b * math.cos(alfa)
        return d_t + self.p('lookahead') * math.sin(alfa), alfa

    def distancia_frente(self, msg, r):
        """Distancia libre mínima entre el frente del chasis y un obstáculo (cono frontal)."""
        ang = msg.angle_min + np.arange(len(r)) * msg.angle_increment
        cono = np.abs(ang) < math.radians(self.p('cono_frente_deg'))
        ok = cono & np.isfinite(r) & (r >= msg.range_min)
        if not np.any(ok):
            return math.inf
        return max(float(np.min(r[ok])) - self.p('semilargo'), 0.0)

    # ------------------------------------------------------------------
    def cb_scan(self, msg):
        r = np.asarray(msg.ranges, dtype=float)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9   # tiempo simulado
        if self.ancho is None:
            self.ancho = self.p('ancho_inicial')

        # 1) Error lateral e = D_des - D_{t+1}  (convención: pared izquierda)
        izq = self.distancia_pared(msg, r, IZQ)
        der = self.distancia_pared(msg, r, DER)
        if izq and der:
            modo = 'centro'
            self.ancho = 0.9 * self.ancho + 0.1 * (izq[0] + der[0])   # filtro paso bajo
            d_des = (izq[0] + der[0]) / 2.0
            e = d_des - izq[0]
        elif izq:
            modo = 'pared izq'
            e = self.ancho / 2.0 - izq[0]
        elif der:
            modo = 'pared der'
            e = -(self.ancho / 2.0 - der[0])   # espejo: demasiado cerca de la derecha -> e < 0
        else:
            modo = 'sin paredes'
            e = 0.0

        # 2) PID (dt con el reloj simulado del scan)
        dt = None if self.t_ant is None else t - self.t_ant
        if dt is None or dt <= 0.0 or dt > 0.5 or modo != self.modo_ant or modo == 'sin paredes':
            # Arranque, salto de tiempo o cambio de modo: sin derivada (evita "patadas")
            self.de_filtrada = 0.0
            if modo == 'sin paredes':
                self.integral = 0.0
        else:
            imax = self.p('integral_max')
            self.integral = min(max(self.integral + e * dt, -imax), imax)
            self.de_filtrada = 0.7 * self.de_filtrada + 0.3 * (e - self.e_ant) / dt
        self.e_ant, self.t_ant, self.modo_ant = e, t, modo

        u = self.p('kp') * e + self.p('ki') * self.integral + self.p('kd') * self.de_filtrada
        w_max = self.p('w_max')
        w = min(max(-u, -w_max), w_max)

        # 3) Velocidad: menor en curvas y al acercarse a una pared de frente
        v = self.p('v_max') * (1.0 - self.p('k_curva') * abs(w) / w_max)
        d_frente = self.distancia_frente(msg, r)
        v = min(v, self.p('k_frente') * d_frente)
        v = min(max(v, self.p('v_min')), self.p('v_max'))

        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.pub_cmd.publish(cmd)
        self.pub_error.publish(Float32(data=float(e)))

        self.get_logger().info(
            f'[{modo}] e = {e:+.3f} m | ω = {w:+.2f} rad/s | v = {v:.2f} m/s | '
            f'ancho = {self.ancho:.2f} m | frente = {d_frente:.2f} m',
            throttle_duration_sec=1.0)


def main():
    rclpy.init()
    nodo = SeguidorPared()
    try:
        rclpy.spin(nodo)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        nodo.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
