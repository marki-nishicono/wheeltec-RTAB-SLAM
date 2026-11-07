#!/usr/bin/env python3
"""Rectify RGB and registered depth together, bound range and data rate.

Depth must ALREADY be hardware registered into the RGB pixel geometry.
No RGB resize without simultaneous intrinsics update; nearest depth remap.
"""
import copy
import time
import cv2
import numpy as np
import rospy
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge


class Frontend:
    def __init__(self):
        self.bridge = CvBridge()
        self.last = 0.0
        self.maps = None
        self.key = None
        self.rate = rospy.get_param('~rate', 5.0)
        self.calibration = rospy.get_param('~calibration', None)
        self.pub_rgb = rospy.Publisher('/lab/rgb/image', Image, queue_size=1)
        self.pub_depth = rospy.Publisher('/lab/depth/image', Image, queue_size=1)
        self.pub_info = rospy.Publisher('/lab/rgb/camera_info', CameraInfo, queue_size=1)
        self.subs = [message_filters.Subscriber(t, cls, queue_size=2) for t, cls in [
            ('/camera/rgb/image_raw', Image), ('/camera/depth_registered/image_raw', Image),
            ('/camera/rgb/camera_info', CameraInfo)]]
        self.sync = message_filters.ApproximateTimeSynchronizer(self.subs, 8, 0.06)
        self.sync.registerCallback(self.receive)

    def receive(self, rgb, depth, info):
        now = time.monotonic()
        if now - self.last < 1.0 / self.rate:
            return
        self.last = now
        try:
            color = self.bridge.imgmsg_to_cv2(rgb, 'bgr8')
            d = self.bridge.imgmsg_to_cv2(depth, 'passthrough').astype(np.float32)
            if depth.encoding == '16UC1':
                d *= 0.001
            elif depth.encoding != '32FC1':
                raise ValueError('Unsupported depth encoding: ' + depth.encoding)
            if d.shape != color.shape[:2] or depth.header.frame_id != rgb.header.frame_id:
                raise ValueError('Require depth registered into RGB pixels and frame')
            # The vendor driver falls back to generic intrinsics when capture
            # resolution differs from its calibration file. Scale the stored
            # calibration explicitly instead of silently using that fallback.
            if self.calibration:
                info = copy.deepcopy(info)
                cal = self.calibration
                sx, sy = rgb.width/cal['image_width'], rgb.height/cal['image_height']
                k = np.array(cal['camera_matrix']['data'],dtype=float).reshape(3,3)
                k[0,:] *= sx; k[1,:] *= sy
                info.K = k.reshape(-1).tolist()
                info.D = list(cal['distortion_coefficients']['data'])
                info.width, info.height = rgb.width, rgb.height
            key = (tuple(info.K), tuple(info.D), info.width, info.height)
            if self.key != key:
                k = np.array(info.K).reshape(3, 3)
                if k[0, 0] <= 0 or k[1, 1] <= 0:
                    raise ValueError('Invalid camera calibration')
                self.maps = cv2.initUndistortRectifyMap(k, np.array(info.D), np.eye(3), k,
                                                       (info.width, info.height), cv2.CV_32FC1)
                self.key = key
            color = cv2.remap(color, *self.maps, cv2.INTER_LINEAR)
            d = cv2.remap(d, *self.maps, cv2.INTER_NEAREST, borderValue=0)
            d[(~np.isfinite(d)) | (d < 0.5) | (d > 2.0)] = 0
            rgb_out = self.bridge.cv2_to_imgmsg(color, 'bgr8')
            dep_out = self.bridge.cv2_to_imgmsg(d, '32FC1')
            # Preserve source timestamps: never disguise RGB-depth time skew.
            rgb_out.header, dep_out.header = rgb.header, depth.header
            ci = copy.deepcopy(info)
            ci.header = rgb.header
            ci.D = [0.0] * len(ci.D)
            ci.R = np.eye(3).reshape(-1).tolist()
            ci.P = np.column_stack((np.array(ci.K).reshape(3, 3), np.zeros(3))).reshape(-1).tolist()
            self.pub_info.publish(ci)
            self.pub_rgb.publish(rgb_out)
            self.pub_depth.publish(dep_out)
        except Exception as e:
            rospy.logerr_throttle(5, 'RGBD preprocessing: %s', e)


if __name__ == '__main__':
    rospy.init_node('lab_rgbd')
    Frontend()
    rospy.spin()
