#!/usr/bin/env python3
"""Records inputs, results and reference during a run (not in the estimation loop).

Writes CSV files to `out_dir`:
  result_velocity.csv   stamp, velocity, recv_wall
  result_position.csv   stamp, frame, x, y, z, yaw, vx, cov_xx, cov_yy, twist_cov_vv, recv_wall
  inputs.csv            topic, stamp, value, recv_wall
  reference.csv         stamp, x, y, z, v   (/localization/kinematic_state, if present in the bag)
  gnss.csv              topic, stamp, lat, lon, alt, status, vx, vy, recv_wall
  status.csv            recv_wall, json   (/result/slip_status)
  resources.csv         wall, cpu_percent, rss_mb   (tram_odometry_node process, Linux /proc)
End-to-end latency = recv_wall(output with stamp S) - recv_wall(input with stamp S).
Stops by itself `idle_timeout` seconds after the last input message (bag finished).
"""
import math
import os
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from tram_vehicle_msgs.msg import VelocitySensor

try:
    from tram_vehicle_msgs.msg import DriverControllerCommand
except ImportError:  # checker's message package has no DriverControllerCommand
    DriverControllerCommand = None


def stamp(h):
    return h.stamp.sec + 1e-9 * h.stamp.nanosec


class Recorder(Node):
    def __init__(self):
        super().__init__("result_recorder")
        self.out_dir = self.declare_parameter("out_dir", "/tmp/tram_odometry_run").value
        self.idle_timeout = self.declare_parameter("idle_timeout", 5.0).value
        os.makedirs(self.out_dir, exist_ok=True)
        self.f = {n: open(os.path.join(self.out_dir, n + ".csv"), "w") for n in
                  ("result_velocity", "result_position", "inputs", "reference", "gnss", "status", "resources")}
        self.f["result_velocity"].write("stamp,velocity,recv_wall\n")
        self.f["result_position"].write("stamp,frame,x,y,z,yaw,vx,cov_xx,cov_yy,twist_cov_vv,recv_wall\n")
        self.f["inputs"].write("topic,stamp,value,recv_wall\n")
        self.f["reference"].write("stamp,x,y,z,v\n")
        self.f["gnss"].write("topic,stamp,lat,lon,alt,status,vx,vy,recv_wall\n")
        self.f["status"].write("recv_wall;json\n")
        self.f["resources"].write("wall,cpu_percent,rss_mb\n")
        q = qos_profile_sensor_data
        self.last_input = None
        self.create_subscription(VelocitySensor, "/vehicle/front_bogie_velocity",
                                 lambda m: self.inp("front", m.header, m.velocity), q)
        self.create_subscription(VelocitySensor, "/vehicle/rear_bogie_velocity",
                                 lambda m: self.inp("rear", m.header, m.velocity), q)
        if DriverControllerCommand is not None:
            self.create_subscription(DriverControllerCommand, "/vehicle/driver_position_cmd",
                                     lambda m: self.inp("cmd", m.header, m.position), q)
        self.create_subscription(VelocitySensor, "/result/velocity", self.on_vel, q)
        self.create_subscription(Odometry, "/result/position", self.on_pos, q)
        self.create_subscription(Odometry, "/localization/kinematic_state", self.on_ref, q)
        self.create_subscription(String, "/result/slip_status",
                                 lambda m: self.f["status"].write(f"{time.time():.6f};{m.data}\n"), 10)
        for ant in ("master", "rover"):
            self.create_subscription(NavSatFix, f"/sensing/gnss/{ant}/fix",
                                     lambda m, a=ant: self.f["gnss"].write(
                                         f"{a}_fix,{stamp(m.header):.6f},{m.latitude:.9f},{m.longitude:.9f},"
                                         f"{m.altitude:.3f},{m.status.status},,,{time.time():.6f}\n"), q)
            self.create_subscription(TwistStamped, f"/sensing/gnss/{ant}/vel",
                                     lambda m, a=ant: self.f["gnss"].write(
                                         f"{a}_vel,{stamp(m.header):.6f},,,,,{m.twist.linear.x:.4f},"
                                         f"{m.twist.linear.y:.4f},{time.time():.6f}\n"), q)
        self.pid = None
        self.prev_cpu = None
        self.create_timer(1.0, self.on_timer)
        self.get_logger().info(f"recording to {self.out_dir}")

    def inp(self, topic, header, value):
        now = time.time()
        self.last_input = now
        self.f["inputs"].write(f"{topic},{stamp(header):.6f},{value},{now:.6f}\n")

    def on_vel(self, m):
        self.f["result_velocity"].write(f"{stamp(m.header):.6f},{m.velocity:.5f},{time.time():.6f}\n")

    def on_pos(self, m):
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        p = m.pose.pose.position
        self.f["result_position"].write(
            f"{stamp(m.header):.6f},{m.header.frame_id},{p.x:.4f},{p.y:.4f},{p.z:.4f},{yaw:.5f},"
            f"{m.twist.twist.linear.x:.5f},{m.pose.covariance[0]:.5f},{m.pose.covariance[7]:.5f},"
            f"{m.twist.covariance[0]:.6f},{time.time():.6f}\n")

    def on_ref(self, m):
        p = m.pose.pose.position
        self.f["reference"].write(f"{stamp(m.header):.6f},{p.x:.4f},{p.y:.4f},{p.z:.4f},"
                                  f"{m.twist.twist.linear.x:.5f}\n")

    def find_pid(self):
        for d in os.listdir("/proc"):
            if not d.isdigit():
                continue
            try:
                with open(f"/proc/{d}/cmdline", "rb") as f:
                    if b"tram_odometry_node" in f.read():
                        return int(d)
            except OSError:
                pass
        return None

    def on_timer(self):
        if self.pid is None:
            self.pid = self.find_pid()
        if self.pid is not None:
            try:
                with open(f"/proc/{self.pid}/stat") as f:
                    st = f.read().rsplit(")", 1)[1].split()
                cpu = (int(st[11]) + int(st[12])) / os.sysconf("SC_CLK_TCK")
                with open(f"/proc/{self.pid}/status") as f:
                    rss = next(int(l.split()[1]) for l in f if l.startswith("VmRSS")) / 1024.0
                now = time.time()
                if self.prev_cpu is not None:
                    pct = 100.0 * (cpu - self.prev_cpu[1]) / max(now - self.prev_cpu[0], 1e-3)
                    self.f["resources"].write(f"{now:.3f},{pct:.2f},{rss:.2f}\n")
                self.prev_cpu = (now, cpu)
            except (OSError, StopIteration, IndexError):
                self.pid = None
        for fh in self.f.values():
            fh.flush()
        if self.last_input is not None and time.time() - self.last_input > self.idle_timeout:
            self.get_logger().info("inputs idle -> finishing recording")
            raise SystemExit

    def close(self):
        for fh in self.f.values():
            fh.close()


def main():
    rclpy.init()
    node = Recorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
