"""Run one actual watchdog cycle with fake ROS; never contact the vehicle."""
import ast,json,pathlib,threading,types,unittest
source=pathlib.Path('scripts/control_guard.py')
tree=ast.parse(source.read_text())
guard=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='Guard')
class Twist:
    def __init__(self):
        self.linear=types.SimpleNamespace(x=0.)
        self.angular=types.SimpleNamespace(z=0.)
class Publisher:
    def __init__(self):self.messages=[]
    def publish(self,m):self.messages.append(m)

class ManualControl(unittest.TestCase):
    def cycle(self,**changes):
        iterations=iter([False,True])
        ns=dict(json=json,Twist=Twist,rospy=types.SimpleNamespace(is_shutdown=lambda:next(iterations)),
                time=types.SimpleNamespace(monotonic=lambda:100.,sleep=lambda t:None))
        exec(compile(ast.Module(body=[guard],type_ignores=[]),str(source),'exec'),ns)
        g=ns['Guard'].__new__(ns['Guard'])
        g.__dict__.update(lock=threading.Lock(),command=(.1,.2),enabled=True,calibrated=True,
                          watchdog_stops=0,previous_reason=None,max_watch_gap=0.,watch_stamp=0.,wheel_time=100.,scan_time=100.,last=100.,front=.02,back=.02,map_seen=100.,map_locked=False,
                          pub=Publisher(),status=Publisher(),session='test',wheelbase=.143)
        g.__dict__.update(changes);g.watch()
        return g.pub.messages[-1],json.loads(g.status.messages[-1])
    def test_distance_does_not_stop_manual_driving(self):
        m,s=self.cycle();self.assertEqual(m.linear.x,.1);self.assertFalse(s['obstacle_stop_enabled'])
    def test_no_returns_do_not_stop_manual_driving(self):
        m,_=self.cycle(front=0.,back=0.);self.assertEqual(m.linear.x,.1)
    def test_release_watchdog_still_stops(self):
        m,s=self.cycle(last=99.);self.assertEqual(m.linear.x,0);self.assertEqual(s['reason'],'deadman_timeout')
    def test_disconnect_still_stops(self):
        for key in ['wheel_time','scan_time']:
            m,_=self.cycle(**{key:99.});self.assertEqual(m.linear.x,0)
    def test_map_transition_locks_drive(self):
        m,s=self.cycle(map_locked=True);self.assertEqual(m.linear.x,0);self.assertEqual(s['reason'],'map_transition')
    def test_map_manager_crash_stops_drive(self):
        m,s=self.cycle(map_seen=97.);self.assertEqual(m.linear.x,0);self.assertEqual(s['reason'],'map_session_offline')

if __name__=='__main__':unittest.main()
