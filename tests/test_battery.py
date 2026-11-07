import pathlib,sys,unittest
sys.path.insert(0,str(pathlib.Path('scripts').resolve()))
from battery_monitor import BatteryMonitor
class Battery(unittest.TestCase):
    def setUp(self):
        self.t=0.;self.b=BatteryMonitor(clock=lambda:self.t)
    def feed(self,t,v):
        self.t=t;self.b.update(v);return self.b.snapshot()
    def test_continuous_low_voltage_exceeds_twenty_seconds(self):
        for t in range(21):self.assertFalse(self.feed(t,9.8)['warning'])
        self.assertTrue(self.feed(20.1,9.8)['warning'])
        self.assertFalse(self.feed(20.2,9.9)['warning'])
    def test_disconnect_is_not_twenty_seconds_low(self):
        self.feed(0,9.8);self.t=25
        self.assertFalse(self.b.snapshot()['warning']);self.assertFalse(self.b.snapshot()['fresh'])
        self.assertEqual(self.feed(26,9.8)['low_seconds'],0)
    def test_short_dips_and_invalid_data_reset(self):
        for t in range(10):self.feed(t,9.8)
        self.feed(10,10.5);self.assertEqual(self.feed(11,9.8)['low_seconds'],0)
        for v in (float('nan'),float('inf'),0,-1):
            self.assertFalse(self.feed(12,v)['fresh'])
    def test_no_reading_is_unknown(self):
        self.assertIsNone(self.b.snapshot()['voltage'])
if __name__=='__main__':unittest.main()
