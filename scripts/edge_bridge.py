#!/usr/bin/env python3
"""Headless ROS data server. Binds loopback; SSH forwards it to local renderer.

All point projection and map transformations happen on the vehicle. Retains
per-keyframe base-frame clouds so graph changes also correct historical clouds.
"""
import base64
import collections
import gzip
import json
import math
import os
import pathlib
import secrets
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import cv2
import numpy as np
import rospy
import message_filters
import tf2_ros
from tf.transformations import quaternion_matrix
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo, LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String, Float32
from rtabmap_msgs.msg import MapData, Info
sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from map_sessions import MapSessions
from video_stream import VideoStream
from control_socket import ControlSocket
from battery_monitor import BatteryMonitor


def matrix(p):
    q = p.orientation if hasattr(p, 'orientation') else p.rotation
    t = p.position if hasattr(p, 'position') else p.translation
    m = quaternion_matrix([q.x, q.y, q.z, q.w])
    m[:3, 3] = [t.x, t.y, t.z]
    return m


class Bridge:
    def __init__(self):
        self.lock = threading.RLock()
        self.control_lock = threading.RLock()
        self.control_live = {}; self.control_seen = {}
        self.battery = BatteryMonitor()
        self.cv = CvBridge()
        self.tf = tf2_ros.Buffer(cache_time=rospy.Duration(15))
        self.listener = tf2_ros.TransformListener(self.tf)
        self.drive = rospy.Publisher('/lab/drive', TwistStamped, queue_size=1, tcp_nodelay=True)
        self.state = {'backend': 'RTAB-Map 0.21.13', 'mode': 'mapping', 'control': {}, 'stats': {}}
        self.last_seen = {}; self.frames = collections.deque(maxlen=30)
        self.nodes = collections.OrderedDict(); self.graph = {}; self.links = []
        self.paths = {k: collections.deque(maxlen=5000) for k in ['wheel', 'ekf', 'slam']}
        self.grid = None; self.cloud = []; self.features = {}; self.scan = []
        self.rgb = b''; self.stamp = 0.; self.last_frame = 0.; self.revision = 0
        self.preview_cv = threading.Condition()
        self.raw_rgb = None; self.preview_seq = 0
        self.preview_times = collections.deque(maxlen=90)
        self.raw_times = collections.deque(maxlen=90)
        self.cpu_previous = None
        self.camera_info = None
        self.map_cutoff = 0.
        self.odom_reference = None
        self.sessions = None
        self.video = VideoStream()
        self.seq = 0; self.last_cloud = 0.
        self.leases = collections.OrderedDict()
        self.command_sequences = collections.OrderedDict()
        self.run_dir = os.path.expanduser('~/wheeltec_slam_runtime')
        os.makedirs(self.run_dir + '/measurements', exist_ok=True)
        self.log = open(self.run_dir + '/measurements/telemetry.jsonl', 'a', buffering=1)
        rospy.Subscriber('/rtabmap/mapData', MapData, self.on_mapdata, queue_size=1)
        rospy.Subscriber('/rtabmap/info', Info, self.on_info, queue_size=1)
        rospy.Subscriber('/rtabmap/grid_map', OccupancyGrid, self.on_grid, queue_size=1)
        rospy.Subscriber('/wheel/odom', Odometry, lambda m: self.on_odom('wheel', m), queue_size=1)
        rospy.Subscriber('/odometry/filtered', Odometry, lambda m: self.on_odom('ekf', m), queue_size=1)
        rospy.Subscriber('/lab/control_status', String, self.on_control, queue_size=1)
        rospy.Subscriber('/PowerVoltage', Float32, self.on_battery, queue_size=1)
        rospy.Subscriber('/lab/imu_calibration', String, self.on_calibration, queue_size=1)
        rospy.Subscriber('/scan', LaserScan, self.on_scan, queue_size=1)
        rospy.Subscriber('/camera/rgb/image_raw', Image, self.on_raw_rgb, queue_size=1, buff_size=2**22)
        self.subs = [message_filters.Subscriber(t, cls, queue_size=2) for t, cls in [
            ('/lab/rgb/image', Image), ('/lab/depth/image', Image), ('/lab/rgb/camera_info', CameraInfo)]]
        self.sync = message_filters.ApproximateTimeSynchronizer(self.subs, 8, 0.06)
        self.sync.registerCallback(self.on_rgbd)
        self.sessions = MapSessions(self)
        reference=self.sessions.snapshot().get('imu_map_reference')
        if reference and not self.sessions.snapshot()['locked']:
            self.odom_reference=np.asarray(reference['map_from_odom'])
        threading.Thread(target=self.preview_worker, daemon=True).start()
        threading.Thread(target=self.worker, daemon=True).start()

    def on_raw_rgb(self, msg):
        self.state['camera_capture_size']=[msg.width,msg.height]
        with self.preview_cv:
            self.raw_rgb = msg
            self.raw_times.append(time.monotonic())
            self.preview_cv.notify_all()

    def preview_worker(self):
        # Encode once per arriving frame, shared by all viewers. Never queue
        # old images if the encoder or SSH connection is slow.
        previous = None
        last_jpeg = 0.
        last_video = 0.
        while not rospy.is_shutdown():
            with self.preview_cv:
                self.preview_cv.wait_for(lambda: self.raw_rgb is not previous or rospy.is_shutdown(), timeout=1)
                msg = self.raw_rgb
            if msg is None or msg is previous: continue
            previous = msg
            if not self.video.active:continue
            if time.monotonic()-last_video<.095:continue
            last_video=time.monotonic()
            try:
                rgb = self.cv.imgmsg_to_cv2(msg, 'bgr8')
                rgb = cv2.resize(rgb, (320, 240), interpolation=cv2.INTER_AREA)
                self.video.push(rgb)
                with self.lock:
                    self.seen('preview')
                    self.state['camera_capture_size'] = [msg.width, msg.height]
                with self.preview_cv:
                    self.preview_times.append(time.monotonic())
                    self.preview_seq += 1
                    self.preview_cv.notify_all()
            except Exception as e:
                rospy.logwarn_throttle(5, 'preview: %s', e)

    def resource_metrics(self):
        result = {'stamp': time.time(), 'cpu_percent': None, 'gpu_percent': None}
        result['video'] = self.video.stats()
        try:
            ticks = [int(x) for x in pathlib.Path('/proc/stat').read_text().splitlines()[0].split()[1:9]]
            total, idle = sum(ticks), ticks[3]+ticks[4]
            if self.cpu_previous:
                dt, di = total-self.cpu_previous[0], idle-self.cpu_previous[1]
                if dt > 0: result['cpu_percent'] = round(100*(1-di/dt), 1)
            self.cpu_previous = total, idle
            mem = {line.split(':')[0]: int(line.split()[1])*1024 for line in pathlib.Path('/proc/meminfo').read_text().splitlines()}
            result.update(ram_total_mb=round(mem['MemTotal']/2**20),
                          ram_used_mb=round((mem['MemTotal']-mem['MemAvailable'])/2**20),
                          ram_percent=round(100*(1-mem['MemAvailable']/mem['MemTotal']),1))
        except (OSError, ValueError, KeyError): pass
        # Jetson nvhost load is per mille; RAM is shared with the GPU.
        try: result['gpu_percent'] = round(int(pathlib.Path('/sys/devices/gpu.0/load').read_text())/10,1)
        except (OSError, ValueError): pass
        with self.preview_cv:
            for key, stamps in [('camera_fps',self.raw_times),('preview_fps',self.preview_times)]:
                result[key] = round((len(stamps)-1)/(stamps[-1]-stamps[0]),1) if len(stamps)>1 and time.monotonic()-stamps[-1]<1 else 0
        return result

    def seen(self, key): self.last_seen[key] = time.monotonic()

    def on_battery(self, m):
        with self.control_lock:self.battery.update(float(m.data))

    def on_control(self, m):
        with self.control_lock:
            self.control_live['control']=json.loads(m.data);self.control_seen['control']=time.monotonic()
        with self.lock:
            self.state['control'] = json.loads(m.data); self.seen('control')

    def on_calibration(self,m):
        with self.lock:
            self.state['imu_calibration']=json.loads(m.data); self.seen('imu_calibration')

    def on_odom(self, key, m):
        if key=='wheel':
            with self.control_lock:
                self.control_live['wheel_speed']=m.twist.twist.linear.x;self.control_seen['wheel']=time.monotonic()
        p = m.pose.pose.position
        with self.lock:
            self.state[key] = [p.x, p.y, p.z]
            self.state[key+'_speed'] = m.twist.twist.linear.x
            self.state[key+'_yaw_rate'] = m.twist.twist.angular.z
            point=np.array([p.x,p.y,p.z,1.])
            if key=='ekf' and self.odom_reference is not None:point=self.odom_reference @ point
            self.paths[key].append([m.header.stamp.to_sec()]+point[:3].tolist())
            self.seen(key)

    def on_info(self, m):
        with self.lock:
            self.state.update(ref_id=m.refId, loop_id=m.loopClosureId,
                              proximity_id=m.proximityDetectionId, wm_nodes=len(m.wmState))
            self.state['stats'] = dict(zip(m.statsKeys, m.statsValues))
            self.seen('rtabmap')
        if self.sessions:self.sessions.on_info(m)

    def on_scan(self, m):
        with self.control_lock:self.control_seen['scan_raw']=time.monotonic()
        with self.lock: self.seen('scan_raw')
        try:
            tr = self.tf.lookup_transform('map', m.header.frame_id, m.header.stamp, rospy.Duration(0.03))
            t = matrix(tr.transform)
        except Exception:
            return
        r = np.asarray(m.ranges); a = m.angle_min + np.arange(len(r)) * m.angle_increment
        ok = np.isfinite(r) & (r >= max(m.range_min, .15)) & (r <= min(m.range_max, 20))
        xyz = np.column_stack((r[ok]*np.cos(a[ok]), r[ok]*np.sin(a[ok]), np.zeros(ok.sum())))
        xyz = xyz[::2] @ t[:3, :3].T + t[:3, 3]
        with self.lock:
            self.scan = np.round(xyz, 3).tolist(); self.seen('scan')

    def on_rgbd(self, rgb, depth, info):
        now = time.monotonic()
        try:
            color = self.cv.imgmsg_to_cv2(rgb, 'bgr8')
            dep = self.cv.imgmsg_to_cv2(depth, 'passthrough')
            t = matrix(self.tf.lookup_transform('base_footprint', rgb.header.frame_id, rgb.header.stamp,
                                               rospy.Duration(.1)).transform)
            v, u = np.mgrid[0:dep.shape[0]:8, 0:dep.shape[1]:8]
            z = dep[v, u]; ok = np.isfinite(z) & (z >= .5) & (z <= 2.)
            xyz = np.column_stack(((u[ok]-info.K[2])*z[ok]/info.K[0],
                                   (v[ok]-info.K[5])*z[ok]/info.K[4], z[ok]))
            xyz = xyz @ t[:3, :3].T + t[:3, 3]
            colors = color[v[ok], u[ok], ::-1]
            jpeg = cv2.imencode('.jpg', color, [cv2.IMWRITE_JPEG_QUALITY, 65])[1].tobytes()
            f = dict(stamp=rgb.header.stamp.to_sec(), xyz=xyz, colors=colors, jpeg=jpeg,
                     width=rgb.width, height=rgb.height)
            camera = dict(frame=rgb.header.frame_id, k=list(info.K), width=rgb.width, height=rgb.height)
            try:
                camera['map_from_camera'] = matrix(self.tf.lookup_transform('map',rgb.header.frame_id,rospy.Time(0),rospy.Duration(.01)).transform).round(7).tolist()
            except Exception: pass
            with self.lock:
                self.frames.append(f); self.seen('rgbd')
                self.camera_info = camera
                if 'map_from_camera' in camera:
                    self.state['camera_view'] = camera; self.seen('camera_pose')
                self.state['depth_valid_fraction'] = round(float(ok.mean()), 3)
                self.state['rgbd_skew_ms'] = round(abs((rgb.header.stamp-depth.header.stamp).to_sec())*1000, 2)
                self.state['rgbd_callback_ms'] = round((time.monotonic()-now)*1000, 2)
        except Exception as e:
            rospy.logwarn_throttle(5, 'bridge RGBD: %s', e)

    def on_grid(self, m):
        # Occupancy origin includes orientation; send full transform.
        with self.lock:
            if m.header.stamp.to_sec()<self.map_cutoff:return
            self.grid = dict(w=m.info.width, h=m.info.height, resolution=m.info.resolution,
                             origin=matrix(m.info.origin).round(6).tolist(),
                             data=base64.b64encode(np.asarray(m.data, dtype=np.int8).tobytes()).decode())
            self.revision += 1; self.seen('grid')

    def on_mapdata(self, m):
        with self.lock:
            if m.header.stamp.to_sec()<self.map_cutoff:return
            self.graph = {int(i): matrix(p) for i, p in zip(m.graph.posesId, m.graph.poses)}
            self.links = [dict(a=l.fromId, b=l.toId, type=l.type) for l in m.graph.links]
            for n in m.nodes:
                if n.id <= 0: continue
                f = min(self.frames, key=lambda f: abs(f['stamp']-n.stamp)) if self.frames else None
                # Attach only a genuinely synchronized depth sample to each signature.
                if f is None or abs(f['stamp']-n.stamp) > .10: continue
                keypoints = [[k.pt.x, k.pt.y, k.size] for k in n.word_kpts]
                words = {int(i): int(j) for i, j in zip(n.word_id_keys, n.word_id_values)}
                self.nodes[n.id] = dict(f, keypoints=keypoints, words=words, id=n.id)
                self.nodes.move_to_end(n.id)
                while len(self.nodes) > 150: self.nodes.popitem(last=False)
            self.revision += 1; self.seen('graph')

    def initialize_map_reference(self, match):
        # Freeze the accepted map<-odom alignment once. Do not feed SLAM back
        # into the independent velocity/gyro EKF or reset its continuous frame.
        stamp=rospy.Time.from_sec(match['stamp'])
        odom=matrix(self.tf.lookup_transform('odom','base_footprint',stamp,rospy.Duration(1)).transform)
        q=match['yaw']; c,s=math.cos(q),math.sin(q)
        pose=np.array([[c,-s,0,match['x']],[s,c,0,match['y']],[0,0,1,0],[0,0,0,1]])
        reference=pose @ np.linalg.inv(odom)
        with self.lock:
            self.odom_reference=reference
            self.paths['ekf'].clear()
        return dict(match,frame='map',source_frame='odom',map_from_odom=reference.tolist(),
                    method='fixed_alignment_after_localization_and_gyro_calibration')

    def export_map_view(self):
        with self.lock:
            nodes=[]
            for n in self.nodes.values():
                d={k:n[k] for k in ['id','stamp','width','height','keypoints','words']}
                d.update(xyz=n['xyz'].tolist(),colors=n['colors'].tolist(),jpeg=base64.b64encode(n['jpeg']).decode())
                nodes.append(d)
            return dict(nodes=nodes,graph={str(k):v.tolist() for k,v in self.graph.items()},links=self.links,grid=self.grid)

    def reset_map_view(self, view=None):
        view=view or {}
        with self.lock:
            self.map_cutoff=rospy.Time.now().to_sec()
            self.odom_reference=None
            self.nodes.clear();self.frames.clear();self.cloud=[];self.features={};self.scan=[]
            self.graph={int(k):np.asarray(v) for k,v in view.get('graph',{}).items()}
            self.links=view.get('links',[]);self.grid=view.get('grid')
            for n in view.get('nodes',[])[-150:]:
                d=dict(n,xyz=np.asarray(n['xyz']),colors=np.asarray(n['colors']),jpeg=base64.b64decode(n['jpeg']),
                       words={int(k):int(v) for k,v in n['words'].items()})
                self.nodes[n['id']]=d
            for key in self.paths:self.paths[key].clear()
            for key in ['pose','camera_view','ref_id','loop_id','proximity_id','wm_nodes']:self.state.pop(key,None)
            self.state['stats']={}
            for key in ['rtabmap','grid','graph','scan','camera_pose']:self.last_seen.pop(key,None)
            self.revision+=1

    def make_features(self):
        if not self.nodes: return {}
        current = self.nodes[next(reversed(self.nodes))]
        prior_id = self.state.get('loop_id') or self.state.get('proximity_id')
        if prior_id not in self.nodes:
            ids = list(self.nodes); prior_id = ids[-2] if len(ids)>1 else None
        previous = self.nodes.get(prior_id)
        def export(n):
            d = {k:n[k] for k in ['id','stamp','width','height','keypoints']}
            d['jpeg'] = base64.b64encode(n['jpeg']).decode(); return d
        pairs = []
        if previous:
            for word in current['words'].keys() & previous['words'].keys():
                i, j = current['words'][word], previous['words'][word]
                if 0<=i<len(current['keypoints']) and 0<=j<len(previous['keypoints']):
                    pairs.append([current['keypoints'][i][:2], previous['keypoints'][j][:2]])
        return dict(current=export(current), previous=export(previous) if previous else None,
                    pairs=pairs[:120], match_kind='shared_visual_words_candidates_not_verified_inliers',
                    accepted_loop_id=self.state.get('loop_id', 0))

    def worker(self):
        while not rospy.is_shutdown():
            started = time.monotonic()
            resources = self.resource_metrics()
            try:
                tr = self.tf.lookup_transform('map','base_footprint',rospy.Time(0),rospy.Duration(.1))
                p = tr.transform.translation; q = tr.transform.rotation
                pose = [p.x,p.y,p.z,q.x,q.y,q.z,q.w]
                with self.lock:
                    self.state['pose'] = pose
                    self.paths['slam'].append([tr.header.stamp.to_sec(),p.x,p.y,p.z])
            except Exception: pass
            with self.lock: camera = dict(self.camera_info) if self.camera_info else None
            if camera:
                try:
                    tr = self.tf.lookup_transform('map',camera['frame'],rospy.Time(0),rospy.Duration(.03))
                    camera['map_from_camera'] = matrix(tr.transform).round(7).tolist()
                    with self.lock:
                        self.state['camera_view'] = camera
                        self.seen('camera_pose')
                except Exception: pass
            with self.lock:
                self.state['resources'] = resources
                chunks = []
                per_node = max(1, min(1000, 8000//max(len(self.nodes),1)))
                for node_id,n in self.nodes.items():
                    if node_id not in self.graph or not len(n['xyz']): continue
                    step = max(1, math.ceil(len(n['xyz'])/per_node))
                    t = self.graph[node_id]
                    xyz = n['xyz'][::step] @ t[:3,:3].T + t[:3,3]
                    chunks.append(np.column_stack((np.round(xyz,3),n['colors'][::step])))
                self.cloud = np.concatenate(chunks).tolist() if chunks else []
                self.features = self.make_features()
                self.state.update(cloud_points=len(self.cloud), cached_keyframes=len(self.nodes),
                                  graph_nodes=len(self.graph), graph_edges=len(self.links))
                self.state['map_prepare_ms'] = round((time.monotonic()-started)*1000,2)
                self.log.write(json.dumps(dict(timestamp=rospy.Time.now().to_sec(), **self.state), allow_nan=False)+'\n')
            time.sleep(max(.05, 1-(time.monotonic()-started)))

    def control_snapshot(self):
        with self.control_lock:
            token = secrets.token_hex(12)
            self.leases[token] = time.monotonic()
            while len(self.leases)>80:self.leases.popitem(last=False)
            return dict(control=self.control_live.get('control',{}),wheel_speed=self.control_live.get('wheel_speed',0),
                        battery=self.battery.snapshot(),
                        drive_token=token,drive_block=self.drive_block(),
                        ages={k:round(time.monotonic()-self.control_seen.get(k,0),3) for k in ('wheel','scan_raw','control')})

    def get(self, path):
        if path == '/control':return self.control_snapshot()
        if path == '/maps':return dict(maps=self.sessions.store.list(),session=self.sessions.snapshot())
        with self.lock:
            if path == '/state':
                return dict(self.state, map_session=self.sessions.snapshot(),ages={k:round(time.monotonic()-v,2) for k,v in self.last_seen.items()},
                            scan=self.scan, revision=self.revision,
                            trajectories={k:list(v)[::10] for k,v in self.paths.items()})
            if path == '/map':
                return dict(revision=self.revision, cloud=self.cloud, grid=self.grid, links=self.links,
                            graph={str(k):v.round(6).tolist() for k,v in self.graph.items()})
            if path == '/features': return self.features
            return None

    def drive_block(self):
        if self.sessions and self.sessions.snapshot()['locked']:return 'map_transition'
        now = time.monotonic()
        for key, limit in [('control', .8), ('wheel', .6), ('scan_raw', .6)]:
            if now-self.control_seen.get(key, -999) > limit:
                return key+'_offline'
        control = self.control_live.get('control', {})
        if not control.get('enabled'): return 'motion_locked'
        if control.get('reason') in ('imu_calibrating_keep_stationary', 'base_stale', 'scan_stale','map_transition','map_session_offline'):
            return control['reason']
        return None


class Handler(BaseHTTPRequestHandler):
    protocol_version = 'HTTP/1.1'
    disable_nagle_algorithm = True
    bridge = None
    def setup(self):
        super().setup()
        self.connection.settimeout(3)
    def do_GET(self):
        path = self.path.split('?')[0]
        if path in ('/rgb.mjpg','/rgb.jpg'):
            self.send_error(410,'Use on-demand H.264 preview');return
        if path == '/video.h264':
            with self.bridge.preview_cv:
                fresh=bool(self.bridge.raw_times) and time.monotonic()-self.bridge.raw_times[-1]<2
            if not fresh:
                self.send_error(503,'Camera has no fresh frames');return
            self.send_response(200);self.send_header('Content-Type','application/octet-stream')
            self.send_header('Cache-Control','no-store');self.send_header('Connection','close');self.end_headers()
            self.close_connection=True;self.connection.settimeout(1)
            seq=-1
            self.bridge.video.acquire()
            try:
                while not rospy.is_shutdown():
                    frames=self.bridge.video.next(seq)
                    if not frames:break  # Camera silence must release the viewer too.
                    for seq,key,packet in frames:
                        self.wfile.write(packet)
                    self.wfile.flush()
            except OSError:pass
            finally:self.bridge.video.release()
            return
        if path == '/rgb.mjpg':
            self.send_response(200)
            self.send_header('Content-Type','multipart/x-mixed-replace; boundary=frame')
            self.send_header('Cache-Control','no-store'); self.end_headers()
            self.connection.settimeout(2)
            seq = -1
            try:
                while not rospy.is_shutdown():
                    with self.bridge.preview_cv:
                        self.bridge.preview_cv.wait_for(lambda:self.bridge.preview_seq!=seq,timeout=1)
                        if seq == self.bridge.preview_seq: continue
                        seq = self.bridge.preview_seq
                    with self.bridge.lock: data = self.bridge.rgb
                    if not data: continue
                    self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '+str(len(data)).encode()+b'\r\n\r\n'+data+b'\r\n')
                    self.wfile.flush()
            except (OSError, TimeoutError): pass
            return
        if path == '/rgb.jpg':
            with self.bridge.lock: data = self.bridge.rgb
            content_type = 'image/jpeg'
        else:
            obj = self.bridge.get(path)
            if obj is None: self.send_error(404); return
            data = json.dumps(obj, allow_nan=False, separators=(',',':')).encode()
            content_type = 'application/json'
        compressed = 'gzip' in self.headers.get('Accept-Encoding','') and len(data)>1024 and content_type=='application/json'
        if compressed: data = gzip.compress(data, compresslevel=1)
        self.send_response(200); self.send_header('Content-Type',content_type)
        self.send_header('Cache-Control','no-store')
        if compressed: self.send_header('Content-Encoding','gzip')
        self.send_header('Content-Length',str(len(data))); self.end_headers()
        try: self.wfile.write(data)
        except (BrokenPipeError,ConnectionResetError): pass
    def do_POST(self):
        if self.path=='/drive':
            self.send_error(410,'Refresh browser: exclusive WebSocket control required');return
        if self.path == '/map-session':
            if self.headers.get('X-Lab-Control')!='deadman-v1':self.send_error(403);return
            try:
                n=int(self.headers.get('Content-Length',0))
                if not 0<n<1024:raise ValueError('请求大小无效')
                result=self.bridge.sessions.start(json.loads(self.rfile.read(n)))
                code=202
            except (ValueError,KeyError,TypeError) as e:result={'error':str(e)};code=400
            except RuntimeError as e:result={'error':str(e)};code=409
            data=json.dumps(result,ensure_ascii=False).encode()
            self.send_response(code);self.send_header('Content-Type','application/json');self.send_header('Content-Length',str(len(data)))
            self.end_headers();self.wfile.write(data);return
        # Non-simple header blocks browser cross-origin drive requests.
        if self.headers.get('X-Lab-Control') != 'deadman-v1' or self.path!='/drive':
            self.send_error(403); return
        try:
            n = int(self.headers.get('Content-Length',0))
            if not 0<n<1024: raise ValueError('invalid body size')
            d = json.loads(self.rfile.read(n)); v=float(d['speed']); s=float(d['steer'])
            if not math.isfinite(v+s): raise ValueError('nonfinite command')
            with self.bridge.lock:
                client = str(d['client'])[:64]; sequence = int(d['seq'])
                if sequence <= self.bridge.command_sequences.get(client,-1):
                    raise ValueError('out of order command')
                # A stop is always permitted; nonzero requests must carry a recent
                # vehicle-issued lease, preventing queued SSH packets restarting motion.
                if v != 0 and time.monotonic()-self.bridge.leases.get(d.get('token'),-999)>0.55:
                    raise ValueError('expired drive lease')
                if v != 0 and self.bridge.drive_block():
                    self.send_error(409, self.bridge.drive_block()); return
                self.bridge.command_sequences[client] = sequence
                self.bridge.command_sequences.move_to_end(client)
                while len(self.bridge.command_sequences)>20: self.bridge.command_sequences.popitem(last=False)
                t = TwistStamped(); t.header.stamp = rospy.Time.now()
                t.twist.linear.x = v; t.twist.angular.z = s
                self.bridge.drive.publish(t)
            data=json.dumps(self.bridge.control_snapshot(),separators=(',',':')).encode()
            self.send_response(200);self.send_header('Content-Type','application/json')
            self.send_header('Content-Length',str(len(data)));self.send_header('Cache-Control','no-store')
            self.end_headers();self.wfile.write(data)
        except (ValueError, KeyError, TypeError): self.send_error(400)
    def log_message(self,*args): pass


class EdgeServer(ThreadingHTTPServer):
    request_queue_size = 64
    daemon_threads = True


if __name__ == '__main__':
    rospy.init_node('lab_edge_bridge')
    Handler.bridge = Bridge()
    ControlSocket(Handler.bridge).start()
    server = EdgeServer(('127.0.0.1',8765),Handler)
    control_server = EdgeServer(('127.0.0.1',8766),Handler)
    threading.Thread(target=control_server.serve_forever,daemon=True).start()
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever,daemon=True).start()
    rospy.spin()
    server.shutdown()
