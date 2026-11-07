#!/usr/bin/env python3
"""Vehicle-side map lifecycle. Saved snapshots are immutable; work on copies.

RTAB-Map services are the only writers of its active SQLite database. All map
transitions hold the independent drive guard through a heartbeat topic.
"""
import collections
import datetime
import gzip
import json
import math
import os
import pathlib
import shutil
import sqlite3
import threading
import time
import uuid


def atomic_json(path, data):
    temporary = pathlib.Path(str(path)+'.tmp')
    with temporary.open('w') as f:
        json.dump(data, f, ensure_ascii=False, allow_nan=False, indent=2)
        f.flush(); os.fsync(f.fileno())
    os.replace(temporary, path)


def inspect_database(path):
    with sqlite3.connect(path.resolve().as_uri()+'?mode=ro', uri=True, timeout=5) as db:
        if db.execute('PRAGMA quick_check').fetchone()[0] != 'ok':
            raise ValueError('地图数据库完整性检查失败')
        ids = [row[0] for row in db.execute('SELECT id FROM Node WHERE id>0')]
        return dict(node_count=len(ids), node_ids=ids, bytes=path.stat().st_size)


def checked_label(value):
    label = str(value or '').strip() or datetime.datetime.now().strftime('地图 %Y-%m-%d %H:%M:%S')
    if len(label) > 80 or any(ord(c)<32 for c in label): raise ValueError('地图名称须为 1–80 个可见字符')
    return label


class MapStore:
    def __init__(self, root):
        self.root = pathlib.Path(root).resolve()
        self.saved = self.root/'saved'; self.working = self.root/'working'
        self.saved.mkdir(parents=True, exist_ok=True); self.working.mkdir(parents=True, exist_ok=True)

    def manifest(self, identifier):
        if not isinstance(identifier,str) or len(identifier)!=32 or any(c not in '0123456789abcdef' for c in identifier):
            raise ValueError('无效地图 ID')
        path = self.saved/(identifier+'.json')
        if not path.is_file(): raise ValueError('找不到已保存地图')
        data = json.loads(path.read_text())
        if data['id'] != identifier: raise ValueError('地图元数据不匹配')
        return data

    def list(self):
        entries = []
        for path in self.saved.glob('*.json'):
            try:
                d = self.manifest(path.stem)
                if (self.saved/(d['id']+'.db')).is_file(): entries.append(d)
            except (ValueError, KeyError, OSError): continue
        return sorted(entries, key=lambda d:d['created'], reverse=True)

    def save(self, source, label, view, extra):
        identifier = uuid.uuid4().hex
        dest = self.saved/(identifier+'.db'); temporary = dest.with_suffix('.db.tmp')
        shutil.copy2(source, temporary)
        details = inspect_database(temporary); details.pop('node_ids')
        os.replace(temporary, dest)
        view_path = self.saved/(identifier+'.view.json.gz')
        with gzip.open(str(view_path)+'.tmp','wt') as f: json.dump(view,f,allow_nan=False)
        os.replace(str(view_path)+'.tmp', view_path)
        data = dict(id=identifier, label=checked_label(label), created=time.time(), **details, **extra)
        atomic_json(self.saved/(identifier+'.json'), data)
        return data

    def copy_for_resume(self, identifier):
        data = self.manifest(identifier)
        source = self.saved/(identifier+'.db')
        details = inspect_database(source)
        if not details['node_count']: raise ValueError('该地图没有可定位节点')
        dest = self.working/(uuid.uuid4().hex+'.db')
        shutil.copy2(source,dest)
        view_path = self.saved/(identifier+'.view.json.gz')
        view = json.load(gzip.open(view_path,'rt')) if view_path.is_file() else {}
        return data, dest, set(details['node_ids']), view

    def delete(self, identifier):
        data=self.manifest(identifier)
        # Only saved snapshots: the active working copy is independent.
        for suffix in ('.db','.view.json.gz','.json'):
            path=self.saved/(identifier+suffix)
            if path.exists():path.unlink()
        return data


class MatchGate:
    """Accept only fresh, stable matches to nodes present in the saved map."""
    def __init__(self, saved_ids, after_stamp):
        self.saved_ids = set(saved_ids); self.after = after_stamp
        self.infos = {}; self.last = None; self.count = 0; self.accepted = None

    def info(self, stamp, loop_id, proximity_id):
        node = loop_id or proximity_id
        if stamp > self.after and node in self.saved_ids:
            self.infos[round(stamp,4)] = (node,'visual_loop' if loop_id else 'geometric_proximity')
        self.infos = dict(sorted(self.infos.items())[-10:])

    def pose(self, stamp, xyz_yaw, covariance):
        match = self.infos.get(round(stamp,4))
        if not match or stamp <= self.after or not all(math.isfinite(x) for x in xyz_yaw+covariance): return
        if not (0 <= covariance[0] <= .25 and 0 <= covariance[1] <= .25 and 0 <= covariance[2] <= .12): return
        if self.last and stamp <= self.last[0]: return
        stable = self.last and stamp-self.last[0]<3 and math.hypot(xyz_yaw[0]-self.last[1][0],xyz_yaw[1]-self.last[1][1])<.15 and abs(math.atan2(math.sin(xyz_yaw[2]-self.last[1][2]),math.cos(xyz_yaw[2]-self.last[1][2])))<.15
        self.count = self.count+1 if stable else 1
        self.last = (stamp,xyz_yaw)
        if self.count >= 2:
            self.accepted = dict(stamp=stamp,node_id=match[0],kind=match[1],x=xyz_yaw[0],y=xyz_yaw[1],yaw=xyz_yaw[2],confirmed_frames=self.count)


class MapSessions:
    def __init__(self, bridge):
        import rospy
        from std_msgs.msg import String
        from geometry_msgs.msg import PoseWithCovarianceStamped
        self.rospy=rospy; self.bridge=bridge; self.lock=threading.RLock()
        self.store=MapStore(pathlib.Path(bridge.run_dir)/'maps')
        self.state_file=self.store.root/'session_state.json'
        self.active=pathlib.Path(rospy.get_param('/rtabmap/rtabmap/database_path')).resolve()
        self.match_gate=None; self.poses={}; self.requests=collections.OrderedDict()
        self.status=dict(phase='mapping',message='正在建图',locked=False,busy=False,epoch=uuid.uuid4().hex,
                         active_database=self.active.name,active_label=self.active.stem,operation_id=None,error=None)
        boot_id=pathlib.Path('/proc/sys/kernel/random/boot_id').read_text().strip()
        if self.state_file.is_file():
            previous=json.loads(self.state_file.read_text())
            if previous.get('active_path')==str(self.active):
                for key in ('active_label','source_map_id','last_saved','imu_map_reference','match','mode'):
                    if key in previous:self.status[key]=previous[key]
            if previous.get('locked'):
                self.status.update(phase='error',locked=True,message='上次地图操作未完成，请重新选择旧地图或新建',error='interrupted_operation')
            if previous.get('boot_id')!=boot_id:
                self.status.pop('imu_map_reference',None);self.status.pop('match',None)
                self.status.update(phase='error',locked=True,message='车载电脑重启，需从已保存地图重新匹配定位或新建地图',error='relocalization_required')
        self.status['boot_id']=boot_id
        if str(rospy.get_param('/rtabmap/rtabmap/Mem/IncrementalMemory','true')).lower()=='false':
            self.status.update(phase='error',locked=True,mode='localization',message='定位模式已恢复，请选择已保存地图继续，或完全重新建图')
        self.pub=rospy.Publisher('/lab/map_session',String,queue_size=1,latch=True)
        self.sub=rospy.Subscriber('/rtabmap/localization_pose',PoseWithCovarianceStamped,self.on_pose,queue_size=5)
        threading.Thread(target=self.heartbeat,daemon=True).start()

    def snapshot(self):
        with self.lock: return dict(self.status)

    def set(self, **values):
        with self.lock:
            self.status.update(values)
            atomic_json(self.state_file,dict(self.status,active_path=str(self.active)))

    def heartbeat(self):
        while not self.rospy.is_shutdown():
            self.pub.publish(json.dumps(self.snapshot(),ensure_ascii=False))
            time.sleep(.3)

    def call(self, name, *args):
        from std_srvs.srv import Empty,Trigger
        from rtabmap_msgs.srv import LoadDatabase,PublishMap
        from robot_localization.srv import SetPose
        services={'load_database':LoadDatabase,'publish_map':PublishMap,'/lab/imu_recalibrate':Trigger,'/set_pose':SetPose}
        path=name if name.startswith('/') else '/rtabmap/'+name
        self.rospy.wait_for_service(path,timeout=8)
        return self.rospy.ServiceProxy(path,services.get(name,Empty))(*args)

    def configure(self, mapping, all_nodes=False):
        # set_mode_* alone does not update the wrapper's parameters_ used by
        # load_database/backup. Set private params and update them explicitly.
        self.rospy.set_param('/rtabmap/rtabmap/Mem/IncrementalMemory','true' if mapping else 'false')
        self.rospy.set_param('/rtabmap/rtabmap/Mem/InitWMWithAllNodes','true' if all_nodes else 'false')
        for key in ('RGBD/LinearUpdate','RGBD/AngularUpdate'):
            self.rospy.set_param('/rtabmap/rtabmap/'+key,'0.1' if mapping else '0.0')
        self.call('update_parameters')

    def start(self, request):
        action=request.get('action'); request_id=request.get('request_id')
        if action not in ('save','new','continue','delete'): raise ValueError('未知地图操作')
        if not isinstance(request_id,str) or not 8<=len(request_id)<=80: raise ValueError('无效请求 ID')
        label=checked_label(request.get('label'))
        if action=='continue': self.store.manifest(request.get('map_id'))
        with self.lock:
            if request_id in self.requests: return self.requests[request_id]
            if self.status['busy']: raise RuntimeError('已有地图操作进行中')
            if action=='delete':
                deleted=self.store.delete(request.get('map_id'))
                if (self.status.get('last_saved') or {}).get('id')==deleted['id']:self.status['last_saved']=None
                if self.status.get('source_map_id')==deleted['id']:self.status['source_map_id']=None
                self.set()
                result=dict(accepted=True,deleted_id=deleted['id'],deleted_label=deleted['label'])
                self.requests[request_id]=result
                while len(self.requests)>64:self.requests.popitem(last=False)
                return result
            if action=='save' and self.status['phase']!='mapping': raise RuntimeError('请先完成重定位，或新建地图')
            operation=uuid.uuid4().hex
            self.status.update(busy=True,locked=True,phase='stopping',message='正在停车并等待静止',error=None,
                               operation_id=operation,epoch=uuid.uuid4().hex,match=None,imu_map_reference=None)
            result=dict(accepted=True,operation_id=operation)
            self.requests[request_id]=result
            while len(self.requests)>64:self.requests.popitem(last=False)
        self.set()
        with self.bridge.control_lock:self.bridge.leases.clear()
        threading.Thread(target=self.run,args=(action,label,request.get('map_id')),daemon=True).start()
        return result

    def wait_stationary(self, seconds=1.0, timeout=15):
        deadline=time.monotonic()+timeout; since=None
        while time.monotonic()<deadline:
            with self.bridge.lock:
                fresh=time.monotonic()-self.bridge.last_seen.get('wheel',0)<.4
                still=abs(self.bridge.state.get('wheel_speed',999))<.005 and abs(self.bridge.state.get('wheel_yaw_rate',999))<.01
            if fresh and still:
                if since is None:since=time.monotonic()
                if time.monotonic()-since>=seconds:return
            else:since=None
            time.sleep(.1)
        raise RuntimeError('未检测到持续静止的底盘反馈，请停车后重试')

    def save_current(self,label):
        self.set(phase='saving',message='正在写入数据库并保存独立副本')
        self.call('backup')
        backup=pathlib.Path(str(self.active)+'.back')
        view=self.bridge.export_map_view()
        with self.bridge.lock:
            extra=dict(camera_size=self.bridge.state.get('camera_capture_size'),
                       imu_calibration=self.bridge.state.get('imu_calibration'),source_database=self.active.name)
        data=self.store.save(backup,label,view,extra)
        self.set(last_saved=data)
        return data

    def load(self,path,view):
        self.bridge.reset_map_view()
        self.call('load_database',str(path),False)
        self.active=path
        self.rospy.set_param('/rtabmap/rtabmap/database_path',str(path))
        self.bridge.reset_map_view(view)
        self.set(active_database=path.name)
        self.call('publish_map',True,True,False)

    def recalibrate(self):
        self.set(phase='calibrating',message='定位已确定，正在静止校准 IMU（约 6 秒）')
        response=self.call('/lab/imu_recalibrate')
        if not response.success: raise RuntimeError(response.message)
        deadline=time.monotonic()+30
        while time.monotonic()<deadline:
            with self.bridge.lock: c=dict(self.bridge.state.get('imu_calibration',{}))
            if c.get('generation')==response.message and c.get('calibrated'):
                return c
            time.sleep(.1)
        raise RuntimeError('IMU 静止校准超时，请确保车辆静止后重试')

    def on_info(self,m):
        with self.lock:
            if self.match_gate:
                stamp=m.header.stamp.to_sec()
                self.match_gate.info(stamp,m.loopClosureId,m.proximityDetectionId)
                if round(stamp,4) in self.poses:self.consume_pose(self.poses[round(stamp,4)])

    def consume_pose(self,m):
        p=m.pose.pose; q=p.orientation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        self.match_gate.pose(m.header.stamp.to_sec(),[p.position.x,p.position.y,yaw],
                             [m.pose.covariance[i] for i in (0,7,35)])

    def on_pose(self,m):
        with self.lock:
            self.poses[round(m.header.stamp.to_sec(),4)]=m
            self.poses=dict(sorted(self.poses.items())[-10:])
            if self.match_gate:self.consume_pose(m)

    def await_match(self, after, timeout=60):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            with self.lock: match=self.match_gate.accepted
            if match and match['stamp']>after and self.rospy.Time.now().to_sec()-match['stamp']<3:
                self.wait_stationary(.5,3)
                self.set(match=match)
                return match
            time.sleep(.15)
        raise RuntimeError('未在旧地图中得到连续可靠匹配。请将车放回已扫描区域、保持相机前方 0.5–2 m 有物体，再点击继续重试')

    def run(self,action,label,identifier):
        try:
            self.wait_stationary()
            self.call('pause')
            if action=='save':
                self.save_current(label)
                self.call('resume')
                self.set(phase='mapping',message='地图已保存，可继续建图',busy=False,locked=False)
                return
            # Preserve the active map before every switch, including new-map.
            self.save_current('自动保存 · '+self.status['active_label'])
            if action=='new':
                self.configure(True)
                path=self.store.working/(uuid.uuid4().hex+'.db')
                from geometry_msgs.msg import PoseWithCovarianceStamped
                pose=PoseWithCovarianceStamped();pose.header.frame_id='odom';pose.header.stamp=self.rospy.Time.now()
                pose.pose.pose.orientation.w=1.
                for i in (0,7,14,21,28,35):pose.pose.covariance[i]=.0001
                self.call('/set_pose',pose)
                self.load(path,{})
                self.set(active_label=label,source_map_id=None,mode='mapping')
                self.recalibrate()
                self.call('resume')
            else:
                data,path,ids,view=self.store.copy_for_resume(identifier)
                self.configure(False,True)
                self.load(path,view)
                self.set(active_label=data['label']+' · 续建',source_map_id=identifier,mode='localization',
                         phase='localizing',message='已加载旧地图，正在匹配当前位置；请保持静止')
                start=self.rospy.Time.now().to_sec()
                with self.lock:self.match_gate=MatchGate(ids,start);self.poses.clear()
                self.call('resume')
                self.await_match(start)
                calibration=self.recalibrate()
                with self.lock:
                    self.match_gate=MatchGate(ids,calibration['stamp']);self.poses.clear()
                self.set(phase='verifying',message='IMU 校准完成，正在复核地图位置')
                match=self.await_match(calibration['stamp'])
                reference=self.bridge.initialize_map_reference(match)
                self.call('pause')
                self.configure(True,False)
                self.call('resume')
                self.set(imu_map_reference=reference,mode='mapping')
            with self.lock:self.match_gate=None
            self.set(phase='mapping',message='建图就绪，请重新勾选行驶使能',busy=False,locked=False)
        except Exception as e:
            self.rospy.logerr('Map session: %s',e)
            # Fail closed; keep localization running for diagnostics, but do
            # not silently start mapping after an unverified position.
            self.set(phase='error',message=str(e),error=str(e),busy=False,locked=True)
