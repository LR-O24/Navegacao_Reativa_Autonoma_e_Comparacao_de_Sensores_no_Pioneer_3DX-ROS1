#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Nó de navegação reativa para o Pioneer 3DX.

O sensor de percepção (LIDAR ou sonar) é escolhido pelo parâmetro
~sensor_source, carregado do arquivo config/sensor_config.yaml.

Arquitetura:
    LaserScan / PointCloud --(callback)--> lista de (ângulo, distância)
        --> divisão em setores angulares (mínimo por setor)
        --> campos potenciais (atração frontal + repulsão por setor)
        --> velocidades Twist em ~cmd_vel_topic

Como ambos os sensores são convertidos para o mesmo formato e reduzidos
ao mesmo conjunto de setores, a lógica de controle é idêntica e o único
ponto que muda entre os modos é o subscriber/callback.
"""
import math
import time

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, PointCloud


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class ReactiveAvoider(object):
    def __init__(self):
        g = rospy.get_param

        self.source = str(g("~sensor_source", "lidar")).strip().lower()
        self.cmd_topic = g("~cmd_vel_topic", "/RosAria/cmd_vel")

        # Percepção
        self.max_range = float(g("~max_range", 1.5))
        self.fov = math.radians(float(g("~fov_deg", 200.0)))
        sector = math.radians(float(g("~sector_deg", 10.0)))
        self.n_sectors = max(1, int(round(self.fov / sector)))
        self.sector_w = self.fov / self.n_sectors
        self.front_half = math.radians(float(g("~front_half_deg", 30.0)))

        # LIDAR
        self.lidar_offset = float(g("~lidar_angle_offset", 0.0))
        self.lidar_invert = bool(g("~lidar_invert", False))
        self.lidar_min_range = float(g("~lidar_min_range", 0.12))

        # Controle
        self.v_max = float(g("~v_max", 0.25))
        self.w_max = float(g("~w_max", 0.8))
        self.k_att = float(g("~k_att", 1.0))
        self.k_rep = float(g("~k_rep", 0.3))
        self.k_w = float(g("~k_w", 1.5))
        self.stop_dist = float(g("~stop_distance", 0.40))
        self.clear_dist = float(g("~clear_distance", self.stop_dist + 0.25))
        self.slow_dist = max(float(g("~slow_distance", 1.0)), self.stop_dist + 0.1)
        self.timeout = float(g("~sensor_timeout", 1.0))
        rate = float(g("~control_rate", 10.0))

        self.obstacles = []            # lista de (angulo, distancia)
        self.last_update = rospy.Time(0)
        self.turn_dir = 0              # 0 = livre, +1 = esquerda, -1 = direita
        self._last_log = {}

        self.pub = rospy.Publisher(self.cmd_topic, Twist)

        if self.source == "lidar":
            topic = g("~lidar_topic", "/scan")
            rospy.Subscriber(topic, LaserScan, self.cb_lidar, queue_size=1)
        elif self.source == "sonar":
            topic = g("~sonar_topic", "/RosAria/sonar")
            rospy.Subscriber(topic, PointCloud, self.cb_sonar, queue_size=1)
        else:
            rospy.logfatal("sensor_source invalido: '%s' (use 'lidar' ou 'sonar')",
                           self.source)
            raise SystemExit(1)

        rospy.loginfo("Fonte de percepcao: %s | topico: %s | saida: %s",
                      self.source.upper(), topic, self.cmd_topic)

        rospy.Timer(rospy.Duration(1.0 / rate), self.control)
        rospy.on_shutdown(self.stop)

    def log_throttled(self, key, period, fn, msg, *args):
        # rospy.*_throttle nao existe no ROS Hydro
        now = time.time()
        if now - self._last_log.get(key, 0.0) >= period:
            self._last_log[key] = now
            fn(msg, *args)

    # ------------------------------------------------------------------
    # Callbacks: convertem a leitura bruta para (angulo, distancia)
    # ------------------------------------------------------------------
    def cb_lidar(self, msg):
        obs = []
        rmin = max(msg.range_min, self.lidar_min_range)
        sign = -1.0 if self.lidar_invert else 1.0
        for i, r in enumerate(msg.ranges):
            if math.isnan(r) or math.isinf(r) or r < rmin or r > msg.range_max:
                continue
            if r > self.max_range:
                continue
            a = sign * (msg.angle_min + i * msg.angle_increment) + self.lidar_offset
            obs.append((wrap(a), r))
        self.obstacles = obs
        self.last_update = rospy.Time.now()

    def cb_sonar(self, msg):
        # RosAria publica um ponto por eco, em coordenadas do robo.
        obs = []
        for pt in msg.points:
            d = math.hypot(pt.x, pt.y)
            if 0.0 < d <= self.max_range:
                obs.append((math.atan2(pt.y, pt.x), d))
        self.obstacles = obs
        self.last_update = rospy.Time.now()

    # ------------------------------------------------------------------
    # Setores angulares (iguais para os dois sensores)
    # ------------------------------------------------------------------
    def bin_sectors(self, obs):
        best = [self.max_range] * self.n_sectors
        half = self.fov / 2.0
        for a, d in obs:
            if abs(a) >= half:
                continue
            i = min(int((a + half) / self.sector_w), self.n_sectors - 1)
            if d < best[i]:
                best[i] = d
        return best

    def sector_center(self, i):
        return -self.fov / 2.0 + (i + 0.5) * self.sector_w

    # ------------------------------------------------------------------
    # Controle por campos potenciais
    # ------------------------------------------------------------------
    def control(self, _):
        cmd = Twist()

        age = (rospy.Time.now() - self.last_update).to_sec()
        if self.last_update == rospy.Time(0) or age > self.timeout:
            self.turn_dir = 0
            self.log_throttled("sem_dados", 2.0, rospy.logwarn,
                               "Sem dados do sensor (%s): robo parado.", self.source)
            self.pub.publish(cmd)
            return

        sectors = self.bin_sectors(self.obstacles)

        fx, fy = self.k_att, 0.0
        front_min = self.max_range
        left, right = 0.0, 0.0
        for i, d in enumerate(sectors):
            a = self.sector_center(i)
            if d < self.max_range:
                mag = self.k_rep * (1.0 / max(d, 0.05) - 1.0 / self.max_range)
                fx -= mag * math.cos(a)
                fy -= mag * math.sin(a)
            if abs(a) <= self.front_half:
                front_min = min(front_min, d)
            if a > 0:
                left += d
            else:
                right += d

        # Maquina de estados simples com histerese para evitar oscilacao
        if self.turn_dir == 0:
            if front_min < self.stop_dist or fx <= 0.0:
                self.turn_dir = 1 if left >= right else -1
        elif front_min > self.clear_dist and fx > 0.0:
            self.turn_dir = 0

        if self.turn_dir != 0:
            state = "GIRA"
            cmd.linear.x = 0.0
            cmd.angular.z = self.turn_dir * self.w_max
        else:
            state = "SEGUE"
            err = math.atan2(fy, fx)
            cmd.angular.z = max(-self.w_max, min(self.w_max, self.k_w * err))
            scale = (front_min - self.stop_dist) / (self.slow_dist - self.stop_dist)
            scale = max(0.2, min(1.0, scale))
            cmd.linear.x = self.v_max * max(0.0, math.cos(err)) * scale

        self.log_throttled("estado", 1.0, rospy.loginfo,
                           "[%s] %s frente=%.2fm v=%.2f w=%.2f",
                           self.source, state, front_min,
                           cmd.linear.x, cmd.angular.z)
        self.pub.publish(cmd)

    def stop(self):
        try:
            self.pub.publish(Twist())
        except Exception:
            pass


if __name__ == "__main__":
    rospy.init_node("reactive_avoider")
    ReactiveAvoider()
    rospy.spin()
