#!/usr/bin/env python3
"""Estimate startup gyro bias only during an observed stationary interval.

Keep /imu raw; publish corrected angular velocity to /imu/calibrated. This
does not calibrate camera-IMU extrinsics, scale, or temperature dependence.
"""
import collections,copy,json,os,time,threading,uuid
import numpy as np
import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse

class Calibration:
    def __init__(self):
        self.samples=collections.deque(maxlen=120);self.wheel_time=0.;self.stationary=False;self.bias=None
        self.lock=threading.RLock();self.recalibrating=True;self.generation=uuid.uuid4().hex
        self.pub=rospy.Publisher('/imu/calibrated',Imu,queue_size=10)
        self.status=rospy.Publisher('/lab/imu_calibration',String,queue_size=1,latch=True)
        rospy.Subscriber('/wheel/odom',Odometry,self.wheel,queue_size=1)
        rospy.Subscriber('/imu',Imu,self.imu,queue_size=10)
        self.status.publish(json.dumps({'calibrated':False,'reason':'waiting_for_6s_stationary','generation':self.generation}))
        self.service=rospy.Service('/lab/imu_recalibrate',Trigger,self.reset)
    def reset(self,request):
        with self.lock:
            self.samples.clear();self.recalibrating=True;self.generation=uuid.uuid4().hex
            self.status.publish(json.dumps({'calibrated':False,'reason':'waiting_for_6s_stationary','generation':self.generation}))
            return TriggerResponse(success=True,message=self.generation)
    def wheel(self,m):
        self.stationary=abs(m.twist.twist.linear.x)<.005 and abs(m.twist.twist.angular.z)<.01
        self.wheel_time=time.monotonic()
    def imu(self,m):
        with self.lock:self.process_imu(m)
    def process_imu(self,m):
        a=np.array([m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z])
        if not np.all(np.isfinite(a)):return
        if self.recalibrating:
            if not self.stationary or time.monotonic()-self.wheel_time>.3:
                self.samples.clear();return
            self.samples.append(a)
            if len(self.samples)<120:return
            data=np.array(self.samples)
            if np.max(data.std(axis=0))>.003 or np.linalg.norm(data.mean(axis=0))>.05:
                self.samples.clear();return
            self.bias=data.mean(axis=0)
            self.recalibrating=False
            result={'calibrated':True,'generation':self.generation,'gyro_bias_rad_s':self.bias.tolist(),'gyro_std_rad_s':data.std(axis=0).tolist(),
                    'samples':120,'stamp':m.header.stamp.to_sec(),'method':'startup_stationary_mean'}
            self.status.publish(json.dumps(result))
            path=os.path.expanduser('~/wheeltec_slam_runtime/imu_bias.json')
            with open(path,'w') as f:json.dump(result,f,indent=2)
            rospy.loginfo('Stationary gyro bias calibrated: %s',self.bias)
        out=copy.deepcopy(m);a-=self.bias
        out.angular_velocity.x,out.angular_velocity.y,out.angular_velocity.z=a.tolist()
        # Avoid the vendor's extremely confident 1e-6 rad^2/s^2 default.
        out.angular_velocity_covariance=[0.000025,0,0,0,0.000025,0,0,0,0.000025]
        self.pub.publish(out)

if __name__=='__main__':
    rospy.init_node('lab_imu_calibration');Calibration();rospy.spin()
