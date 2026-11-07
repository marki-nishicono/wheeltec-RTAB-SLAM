"""Bounded H.264 Annex-B access-unit stream shared by all local viewers."""
import collections, struct, threading, time
import gi
gi.require_version('Gst','1.0')
from gi.repository import Gst
Gst.init(None)

class VideoStream:
    def __init__(self):
        self.cv=threading.Condition();self.frames=collections.deque(maxlen=30);self.seq=0
        self.bytes=collections.deque(maxlen=30);self.started=None;self.error=None
        self.lifecycle=threading.RLock();self.viewers=0;self.pipeline=None
    @property
    def active(self):
        return self.viewers>0
    def acquire(self):
        with self.lifecycle:
            if self.viewers==0:self.start()
            self.viewers+=1
    def release(self):
        with self.lifecycle:
            self.viewers=max(0,self.viewers-1)
            if not self.viewers and self.pipeline is not None:
                # Never hold cv here: NULL waits for the appsink callback.
                self.pipeline.set_state(Gst.State.NULL);self.pipeline=None
                with self.cv:self.frames.clear();self.bytes.clear()
    def start(self):
        self.started=None;self.error=None
        # Orin Nano has no NVENC. Never infer hardware availability from plugin presence.
        self.pipeline=Gst.parse_launch(
            'appsrc name=source is-live=true format=time block=false max-bytes=230400 '
            'caps=video/x-raw,format=BGR,width=320,height=240,framerate=10/1 ! '
            'queue max-size-buffers=1 leaky=downstream ! videoconvert ! '
            'video/x-raw,format=I420 ! x264enc tune=zerolatency speed-preset=ultrafast '
            'bitrate=160 vbv-buf-capacity=100 key-int-max=10 bframes=0 threads=2 byte-stream=true ! '
            'video/x-h264,profile=baseline ! h264parse config-interval=-1 ! '
            'video/x-h264,stream-format=byte-stream,alignment=au ! '
            'appsink name=sink emit-signals=true sync=false max-buffers=2 drop=true')
        self.source=self.pipeline.get_by_name('source')
        self.pipeline.get_by_name('sink').connect('new-sample',self.sample)
        self.pipeline.set_state(Gst.State.PLAYING)
    def push(self,image):
        with self.lifecycle:
            if not self.active:return
            self._push(image)
    def _push(self,image):
        now=time.monotonic()
        if self.started is None:self.started=now
        data=image.tobytes();buf=Gst.Buffer.new_allocate(None,len(data),None);buf.fill(0,data)
        buf.pts=int((now-self.started)*Gst.SECOND);buf.duration=Gst.SECOND//10
        self.source.emit('push-buffer',buf)
        msg=self.pipeline.get_bus().pop_filtered(Gst.MessageType.ERROR)
        if msg:self.error=str(msg.parse_error()[0]);raise RuntimeError(self.error)
    def sample(self,sink):
        sample=sink.emit('pull-sample');buf=sample.get_buffer()
        data=buf.extract_dup(0,buf.get_size());key=not buf.has_flags(Gst.BufferFlags.DELTA_UNIT)
        packet=struct.pack('>IBQ',len(data),int(key),int(buf.pts//1000))+data
        with self.cv:
            self.seq+=1;self.frames.append((self.seq,key,packet));self.bytes.append((time.monotonic(),len(data)))
            self.cv.notify_all()
        return Gst.FlowReturn.OK
    def stats(self):
        with self.cv:
            b=list(self.bytes)
        dt=b[-1][0]-b[0][0] if len(b)>1 else 0
        return dict(active=self.active,viewers=self.viewers,codec='H.264 baseline',encoder='x264 CPU (Orin Nano: no NVENC)',target_kbps=160,
                    kbps=round(sum(n for _,n in b[1:])*8/dt/1000,1) if dt and time.monotonic()-b[-1][0]<2 else 0,
                    fps=round((len(b)-1)/dt,1) if dt and time.monotonic()-b[-1][0]<2 else 0,error=self.error)
    def next(self,seq):
        with self.cv:
            self.cv.wait_for(lambda:bool(self.frames) and self.seq>seq,timeout=2)
            frames=list(self.frames)
            if not frames:return []
            if seq<frames[0][0] or self.seq-seq>10:
                keys=[i for i,f in enumerate(frames) if f[1]]
                return frames[keys[-1]:] if keys else []
            return [f for f in frames if f[0]>seq]
