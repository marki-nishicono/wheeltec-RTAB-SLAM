"""Bounded full-duplex control: fresh leases, single driver, no replay."""
import asyncio,json,math,time,threading
from websockets.legacy.server import serve

class ControlSocket:
    def __init__(self,bridge):
        self.bridge=bridge;self.owner=None;self.owner_stamp=0.;self.last_command=0.;self.max_gap=0.;self.accepted=0
        self.emergency_epoch=0;self.rejected={};self.lifetime_max_gap=0.;self.last_received=0.
    def publish(self,v,s):
        import rospy
        from geometry_msgs.msg import TwistStamped
        m=TwistStamped();m.header.stamp=rospy.Time.now();m.twist.linear.x=v;m.twist.angular.z=s
        self.bridge.drive.publish(m)
    def command(self,ws,d,sequence):
        seq=int(d['seq']);v=float(d['speed']);s=float(d['steer']);now=time.monotonic()
        if seq<=sequence or not all(math.isfinite(x) for x in (v,s)):return sequence,'stale_command'
        with self.bridge.control_lock:
            if d.get('emergency') and v==0:
                self.publish(0,0);self.owner=None;self.last_command=0.
                self.bridge.leases.clear();self.emergency_epoch+=1;return seq,None
            if v and d.get('epoch')!=self.emergency_epoch:return seq,'emergency_disarmed'
            if self.owner is not None and self.owner is not ws and now-self.owner_stamp<.55:return seq,'another_driver'
            if v:
                if now-self.bridge.leases.get(d.get('token'),-999)>.55:return seq,'expired_lease'
                block=self.bridge.drive_block()
                if block:return seq,block
                if self.owner is ws and self.last_command:self.max_gap=max(self.max_gap,now-self.last_command)
                else:self.max_gap=0.
                self.lifetime_max_gap=max(self.lifetime_max_gap,self.max_gap)
                self.owner=ws;self.owner_stamp=now;self.last_command=now;self.accepted+=1
            elif self.owner is not ws:return seq,None
            self.publish(v,s)
            if not v:self.owner=None;self.last_command=0.
        return seq,None
    async def client(self,ws,path):
        sequence=-1;error=None;echo=None
        async def status():
            while True:
                d=self.bridge.control_snapshot()
                d.update(socket_error=error,echo=echo,emergency_epoch=self.emergency_epoch,driver_other=self.owner is not None and self.owner is not ws and time.monotonic()-self.owner_stamp<.55,
                         accepted_commands=self.accepted,max_command_gap_ms=round(self.max_gap*1000,1),
                         lifetime_max_command_gap_ms=round(self.lifetime_max_gap*1000,1),rejected_commands=dict(self.rejected))
                await asyncio.wait_for(ws.send(json.dumps(d,separators=(',',':'))),.5)
                await asyncio.sleep(.1)
        task=asyncio.create_task(status())
        def failed(f):
            if not f.cancelled() and f.exception():asyncio.create_task(ws.close())
        task.add_done_callback(failed)
        try:
            async for raw in ws:
                d=json.loads(raw)
                if d.get('type')=='ping':echo=d.get('stamp');continue
                sequence,error=self.command(ws,d,sequence)
                self.last_received=time.monotonic()
                if error:self.rejected[error]=self.rejected.get(error,0)+1
        except (ValueError,KeyError,TypeError):pass
        finally:
            task.cancel()
            if self.owner is ws:self.publish(0,0);self.owner=None;self.last_command=0.
    async def run(self):
        async with serve(self.client,'127.0.0.1',8767,max_size=2048,max_queue=1,ping_interval=2,ping_timeout=2,close_timeout=.5):
            await asyncio.Future()
    def start(self):threading.Thread(target=lambda:asyncio.run(self.run()),daemon=True).start()
