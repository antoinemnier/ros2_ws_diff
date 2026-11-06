#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

RADIO = 0.05
SEPARACION = 0.25


def envolver(angulo):
    return math.atan2(math.sin(angulo), math.cos(angulo))


def yaw_de_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


class CompararOdom(Node):
    def __init__(self):
        super().__init__('comparar_odom')

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.t_anterior = None
        self.pose_plugin = None

        self.create_subscription(
            JointState, '/joint_states', self.cb_joints, 10
        )
        self.create_subscription(
            Odometry, '/odom', self.cb_odom, 10
        )

        self.pub = self.create_publisher(
            Odometry, '/odom_kinematic', 10
        )

        self.create_timer(1.0, self.mostrar_error)

        self.get_logger().info(
            'Comparacion de /odom_kinematic con /odom'
        )

    def cb_odom(self, msg):
        p = msg.pose.pose.position
        theta = yaw_de_quaternion(msg.pose.pose.orientation)
        self.pose_plugin = (p.x, p.y, theta)

    def cb_joints(self, msg):
        # Lecture par nom : l'ordre du message peut varier
        try:
            indice_d = msg.name.index('joint_rueda_der')
            indice_i = msg.name.index('joint_rueda_izq')

            wd = msg.velocity[indice_d]
            wi = msg.velocity[indice_i]
        except (ValueError, IndexError):
            return

        # Temps des mesures, en secondes
        temps = (
            msg.header.stamp.sec
            + msg.header.stamp.nanosec * 1e-9
        )

        if self.t_anterior is None:
            self.t_anterior = temps
            return

        dt = temps - self.t_anterior
        self.t_anterior = temps

        if dt <= 0.0:
            return

        # Cinématique différentielle : roues -> vitesse du robot
        v = RADIO * (wd + wi) / 2.0
        omega = RADIO * (wd - wi) / SEPARACION

        # Intégration d'Euler dans le repère odom
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta = envolver(self.theta + omega * dt)

        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y

        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)

        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = omega

        self.pub.publish(odom)

    def mostrar_error(self):
        if self.pose_plugin is None or self.t_anterior is None:
            return

        xp, yp, thetap = self.pose_plugin

        erreur_position = math.hypot(self.x - xp, self.y - yp)
        erreur_angle = envolver(self.theta - thetap)

        self.get_logger().info(
            f'Error posicion: {erreur_position:.4f} m | '
            f'Error angular: {erreur_angle:.4f} rad'
        )


def main():
    rclpy.init()
    nodo = CompararOdom()

    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()