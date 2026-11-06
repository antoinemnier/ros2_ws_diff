#!/usr/bin/env python3
"""
vueltas_odom.py - Cronómetro de vueltas con /clock y error de la odometría.

Compara /odom (DiffDrive) con /pose_real (PosePublisher de Gazebo = verdad de terreno).

Marcos:
    /odom empieza en (0, 0, 0) donde nace el robot; /pose_real está en el marco del mundo.
    La transformación (x0, y0, ψ0) se mide con el primer par de mensajes, y luego:
        x_w = x0 + cosψ0·x_o - sinψ0·y_o
        y_w = y0 + sinψ0·x_o + cosψ0·y_o
        ψ_w = ψ0 + ψ_o

Vueltas:
    Cruce de la línea de meta = la pose REAL pasa de x < x_linea a x >= x_linea con |y| < y_linea_max.
    Tras un cruce, el contador solo se rearma cuando el robot pasa por x < x_rearme
    (el otro extremo del circuito): así una oscilación sobre la línea no cuenta dos veces.
    El primer cruce es la salida; los n_vueltas siguientes cierran cada vuelta.

Todos los tiempos vienen de /clock (use_sim_time = True).
Se guarda un CSV con ambas trayectorias para las gráficas del informe.
"""

import csv
import math
import os
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


def yaw(q):
    """Rumbo ψ de un cuaternión que solo gira alrededor de z."""
    return 2.0 * math.atan2(q.z, q.w)


def envolver(a):
    """Ángulo en [-π, π]."""
    return math.atan2(math.sin(a), math.cos(a))


class VueltasOdom(Node):

    def __init__(self):
        super().__init__('vueltas_odom')

        self.declare_parameter('n_vueltas', 3)
        self.declare_parameter('x_linea', 0.0)        # m: línea de salida/meta (x = 0)
        self.declare_parameter('y_linea_max', 1.0)    # m: el cruce solo cuenta en la recta principal
        self.declare_parameter('x_rearme', -2.0)      # m: hay que pasar por aquí para contar otra vuelta
        # Enunciado: odometría del nodo de cinemática (promedio de las 2 ruedas de cada lado)
        self.declare_parameter('topic_odom', 'odom_kinematic')        
        self.declare_parameter('carpeta_csv', os.path.expanduser('~/Robotica/ros2_ws_diff/resultados'))

        gp = self.get_parameter
        self.n_vueltas = gp('n_vueltas').value
        self.x_linea = gp('x_linea').value
        self.y_linea_max = gp('y_linea_max').value
        self.x_rearme = gp('x_rearme').value

        # ---------------- Estado ----------------
        self.odom = None            # última odometría (x, y, ψ) en el marco odom
        self.offset = None          # transformación odom -> mundo (x0, y0, ψ0)
        self.pos_inicial = None
        self.prev = None            # muestra anterior de la pose real (t, x, y)
        self.prev_odom_w = None     # muestra anterior de la odometría en el mundo (x, y)
        self.armado = True          # el primer cruce (salida) cuenta directamente
        self.cruces = []            # tiempos (s, /clock) de cada cruce de la línea
        self.t_arranque = None
        self.dist_real = 0.0
        self.dist_odom = 0.0
        self.terminado = False
        self.n_muestras = 0
        # Rumbo acumulado SIN envolver (rad): permite ver el crecimiento del error vuelta a vuelta
        self.giro_real = 0.0
        self.giro_odom = 0.0
        self.psi_prev = None        # (ψ_real, ψ_odom) de la muestra anterior, envueltos

        # ---------------- CSV ----------------
        carpeta = gp('carpeta_csv').value
        os.makedirs(carpeta, exist_ok=True)
        self.ruta_csv = os.path.join(carpeta, time.strftime('vueltas_%Y%m%d_%H%M%S.csv'))
        self.f_csv = open(self.ruta_csv, 'w', newline='')
        self.csv = csv.writer(self.f_csv)
        self.csv.writerow(['t', 'x_real', 'y_real', 'psi_real',
                           'x_odom', 'y_odom', 'psi_odom', 'vuelta'])

        self.create_subscription(Odometry, gp('topic_odom').value, self.cb_odom, 10)
        self.create_subscription(PoseStamped, 'pose_real', self.cb_pose, 10)
        self.get_logger().info(f'Cronómetro listo: {self.n_vueltas} vueltas | CSV: {self.ruta_csv}')

    # ------------------------------------------------------------------
    def cb_odom(self, msg):
        p = msg.pose.pose
        self.odom = (p.position.x, p.position.y, yaw(p.orientation))

    def odom_en_mundo(self):
        xo, yo, psio = self.odom
        x0, y0, psi0 = self.offset
        c, s = math.cos(psi0), math.sin(psi0)
        return x0 + c * xo - s * yo, y0 + s * xo + c * yo, envolver(psi0 + psio)

    def cb_pose(self, msg):
        if self.odom is None or self.terminado:
            return
        t = self.get_clock().now().nanoseconds * 1e-9      # tiempo simulado (/clock)
        xr, yr = msg.pose.position.x, msg.pose.position.y
        psir = yaw(msg.pose.orientation)

        # 1) Primera muestra: transformación odom -> mundo
        if self.offset is None:
            xo, yo, psio = self.odom
            psi0 = psir - psio
            c, s = math.cos(psi0), math.sin(psi0)
            self.offset = (xr - (c * xo - s * yo), yr - (s * xo + c * yo), psi0)
            self.pos_inicial = (xr, yr)
            self.get_logger().info(
                f'Origen de /odom en el mundo: x0 = {self.offset[0]:+.3f} m, '
                f'y0 = {self.offset[1]:+.3f} m, ψ0 = {math.degrees(psi0):+.1f}°')
        xw, yw, psiw = self.odom_en_mundo()

        # 1b) Acumular el giro sin envolver: cada incremento es pequeño (< 0.2 rad a 30 Hz),
        #     así que envolver(Δψ) es exacto aunque ψ salte de +180° a -180°
        if self.psi_prev is not None:
            self.giro_real += envolver(psir - self.psi_prev[0])
            self.giro_odom += envolver(psiw - self.psi_prev[1])
        self.psi_prev = (psir, psiw)
 
        # 2) Distancias recorridas (real y según la odometría)
        if self.prev is not None:
            self.dist_real += math.hypot(xr - self.prev[1], yr - self.prev[2])
            self.dist_odom += math.hypot(xw - self.prev_odom_w[0], yw - self.prev_odom_w[1])

        # 3) Arranque = primer desplazamiento de más de 1 cm
        if self.t_arranque is None and \
                math.hypot(xr - self.pos_inicial[0], yr - self.pos_inicial[1]) > 0.01:
            self.t_arranque = t
            self.get_logger().info(f'Arranque del robot en t = {t:.2f} s')

        # 4) Rearme y cruce de la línea (con interpolación del instante exacto)
        if xr < self.x_rearme:
            self.armado = True
        if self.prev is not None and self.armado and abs(yr) < self.y_linea_max and \
                self.prev[1] < self.x_linea <= xr:
            t_ant, x_ant = self.prev[0], self.prev[1]
            t_cruce = t_ant + (self.x_linea - x_ant) / (xr - x_ant) * (t - t_ant)
            self.armado = False
            self.registrar_cruce(t_cruce, xr, yr, psir, xw, yw, psiw)

        # 5) CSV a ~10 Hz (una de cada 3 muestras de 30 Hz)
        self.n_muestras += 1
        if self.n_muestras % 3 == 0:
            self.csv.writerow([f'{t:.3f}', f'{xr:.4f}', f'{yr:.4f}', f'{psir:.4f}',
                               f'{xw:.4f}', f'{yw:.4f}', f'{psiw:.4f}',
                               max(len(self.cruces), 0)])

        self.prev = (t, xr, yr)
        self.prev_odom_w = (xw, yw)

    # ------------------------------------------------------------------
    def registrar_cruce(self, t_cruce, xr, yr, psir, xw, yw, psiw):
        self.cruces.append(t_cruce)
        n = len(self.cruces) - 1                       # vueltas completadas
        e_pos = math.hypot(xw - xr, yw - yr)
        e_psi = math.degrees(self.giro_odom - self.giro_real)    # sin envolver: crece vuelta a vuelta

        if n == 0:
            self.get_logger().info(f'SALIDA: cruce de la línea en t = {t_cruce:.2f} s')
            return

        t_vuelta = t_cruce - self.cruces[-2]
        self.get_logger().warn(
            f'VUELTA {n}/{self.n_vueltas}: {t_vuelta:.2f} s | '
            f'error odom: posición {e_pos:.3f} m, rumbo {e_psi:+.1f}° (acumulado) | '
            f'giro real {math.degrees(self.giro_real):+.1f}°, odom {math.degrees(self.giro_odom):+.1f}° | '
            f'distancia real {self.dist_real:.1f} m, odom {self.dist_odom:.1f} m')
        if n >= self.n_vueltas:
            self.terminar(t_cruce, e_pos, e_psi)

    def terminar(self, t_fin, e_pos, e_psi):
        self.terminado = True
        tiempos = [b - a for a, b in zip(self.cruces[:-1], self.cruces[1:])]
        total_linea = t_fin - self.cruces[0]
        total_arranque = t_fin - self.t_arranque if self.t_arranque is not None else float('nan')
        pct = 100.0 * e_pos / self.dist_real if self.dist_real > 0 else float('nan')
        texto = ' | '.join(f'V{i + 1}: {tv:.2f} s' for i, tv in enumerate(tiempos))
        self.get_logger().warn(
            '\n================ RESULTADO ================\n'
            f'  Vueltas: {texto}\n'
            f'  Tiempo total desde la línea:   {total_linea:.2f} s\n'
            f'  Tiempo total desde el arranque: {total_arranque:.2f} s\n'
            f'  Distancia real: {self.dist_real:.2f} m | según odometría: {self.dist_odom:.2f} m\n'
            f'  Error de cierre de la odometría: {e_pos:.3f} m ({pct:.2f} % de la distancia), '
            f'rumbo {e_psi:+.1f}°\n'
            f'  CSV: {self.ruta_csv}\n'
            '===========================================')
        self.f_csv.flush()

    def destroy_node(self):
        self.f_csv.close()
        super().destroy_node()


def main():
    rclpy.init()
    nodo = VueltasOdom()
    try:
        rclpy.spin(nodo)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        nodo.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
