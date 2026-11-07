"""Continuous low-voltage warning; stale telemetry never extends the timer."""
import math
import time

class BatteryMonitor:
    def __init__(self, clock=time.monotonic, threshold=9.9, delay=20., stale_after=2.):
        self.clock=clock;self.threshold=threshold;self.delay=delay;self.stale_after=stale_after
        self.voltage=None;self.last=None;self.low_since=None
    def update(self, voltage):
        now=self.clock()
        if not math.isfinite(voltage) or voltage<=0:
            self.voltage=None;self.last=None;self.low_since=None;return
        if self.last is None or now-self.last>self.stale_after:self.low_since=None
        self.voltage=float(voltage);self.last=now
        if voltage<self.threshold:
            if self.low_since is None:self.low_since=now
        else:self.low_since=None
    def snapshot(self):
        now=self.clock();age=None if self.last is None else max(0.,now-self.last)
        fresh=age is not None and age<=self.stale_after
        if not fresh:self.low_since=None
        duration=max(0.,self.last-self.low_since) if fresh and self.low_since is not None else 0.
        return dict(voltage=self.voltage if fresh else None,last_voltage=self.voltage,age_s=age,
                    fresh=fresh,low_seconds=round(duration,1),warning=duration>self.delay,
                    threshold_v=self.threshold,delay_s=self.delay)
