#!/usr/bin/env python3
"""Independent monotonic watchdog. Only this node publishes base /cmd_vel."""
import json
import math
import threading
import time
import uuid
import rospy
from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from nav_msgs.msg import Odometry


def ackermann(speed, steer, wheelbase):
    if not all(math.isfinite(x) for x in (speed, steer, wheelbase)) or wheelbase <= 0:
        raise ValueError('nonfinite command or invalid wheelbase')
    v = max(-0.12, min(0.12, speed))
    delta = max(-0.35, min(0.35, steer))
    return v, v * math.tan(delta) / wheelbase


class Guard:
    def __init__(self):
        self.lock = threading.Lock()
        self.enabled = rospy.get_param('~motion_enabled', False)
        self.wheelbase = rospy.get_param('~wheelbase', 0.143)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1, tcp_nodelay=True)
        self.status = rospy.Publisher('/lab/control_status', String, queue_size=1)
        self.command = (0., 0.)
        self.watchdog_stops=0;self.previous_reason=None;self.max_watch_gap=0.;self.watch_stamp=0.
        self.last = 0.
        self.scan_time = 0.
        self.wheel_time = 0.
        self.session = uuid.uuid4().hex
        self.calibrated = False
        self.map_locked = True
        self.map_seen = 0.
        self.map_epoch = None
        self.front = self.back = 0.
        rospy.Subscriber('/lab/drive', TwistStamped, self.receive, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/scan', LaserScan, self.scan, queue_size=1)
        rospy.Subscriber('/wheel/odom', Odometry, self.wheel, queue_size=1)
        rospy.Subscriber('/lab/imu_calibration', String, self.calibration, queue_size=1)
        rospy.Subscriber('/lab/map_session', String, self.map_session, queue_size=1)
        rospy.on_shutdown(self.stop)
        threading.Thread(target=self.watch, daemon=True).start()

    def receive(self, msg):
        age = (rospy.Time.now() - msg.header.stamp).to_sec()
        if not 0 <= age < 0.25:
            return
        try:
            v, omega = ackermann(msg.twist.linear.x, msg.twist.angular.z, self.wheelbase)
        except ValueError:
            v, omega = 0., 0.
        with self.lock:
            if v and (self.map_locked or time.monotonic()-self.map_seen > 2):
                return
            self.command = (v, omega)
            self.last = time.monotonic()

    def calibration(self,msg):
        with self.lock: self.calibrated = json.loads(msg.data).get('calibrated',False)

    def wheel(self, msg):
        with self.lock: self.wheel_time = time.monotonic()

    def map_session(self, msg):
        d = json.loads(msg.data)
        with self.lock:
            epoch = d.get('epoch')
            if epoch != self.map_epoch or d.get('locked', True):
                self.command = (0., 0.); self.last = 0.
            if epoch != self.map_epoch: self.session = uuid.uuid4().hex
            self.map_epoch = epoch
            self.map_locked = d.get('locked', True)
            self.map_seen = time.monotonic()

    def scan(self, msg):
        # M10 factory laser is mounted at yaw pi relative to vehicle.
        front, back = [], []
        for i, d in enumerate(msg.ranges):
            if not math.isfinite(d) or d < max(0.15, msg.range_min) or d > msg.range_max:
                continue
            a = msg.angle_min + i * msg.angle_increment + math.pi
            if math.cos(a) > 0.75: front.append(d)
            if math.cos(a) < -0.75: back.append(d)
        with self.lock:
            self.front = min(front) if front else 0.
            self.back = min(back) if back else 0.
            self.scan_time = time.monotonic()

    def stop(self):
        self.pub.publish(Twist())

    def watch(self):
        n = 0
        while not rospy.is_shutdown():
            now = time.monotonic()
            if self.watch_stamp:self.max_watch_gap=max(self.max_watch_gap,now-self.watch_stamp)
            self.watch_stamp=now
            with self.lock:
                v, w = self.command
                reason = 'ready'
                if not self.enabled: reason = 'motion_locked'
                elif now-self.map_seen > 2: reason = 'map_session_offline'
                elif self.map_locked: reason = 'map_transition'
                elif not self.calibrated: reason = 'imu_calibrating_keep_stationary'
                elif now-self.wheel_time > 0.6: reason = 'base_stale'
                elif now-self.scan_time > 0.6: reason = 'scan_stale'
                elif now-self.last > 0.35: reason = 'deadman_timeout'
                if reason != 'ready': v, w = 0., 0.
                if reason=='deadman_timeout' and self.previous_reason=='ready' and self.command[0]:self.watchdog_stops+=1
                self.previous_reason=reason
            m = Twist(); m.linear.x = v; m.angular.z = w
            self.pub.publish(m)
            if n % 5 == 0:
                self.status.publish(json.dumps(dict(enabled=self.enabled, reason=reason, speed=v, yaw_rate=w,
                    requested_speed=self.command[0], front_m=self.front, rear_m=self.back,
                    obstacle_stop_enabled=False, stop_distance_m=None,
                    watchdog_stops=self.watchdog_stops,max_watch_gap_ms=round(self.max_watch_gap*1000,1),
                    session=self.session, wheelbase=self.wheelbase)))
            n += 1
            time.sleep(0.05)


if __name__ == '__main__':
    rospy.init_node('lab_control_guard')
    Guard()
    rospy.spin()
