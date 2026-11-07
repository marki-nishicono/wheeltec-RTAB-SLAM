"""Protocol regression with fake publisher; never connects to a vehicle."""
import sys,pathlib,threading,time,types,unittest
sys.path.insert(0,str(pathlib.Path('scripts').resolve()))
from control_socket import ControlSocket

class Protocol(unittest.TestCase):
    def setUp(self):
        self.b=types.SimpleNamespace(control_lock=threading.RLock(),leases={'fresh':time.monotonic()},drive_block=lambda:None)
        self.c=ControlSocket(self.b);self.sent=[];self.c.publish=lambda v,s:self.sent.append((v,s))
        self.a=object();self.other=object()
    def cmd(self,ws,seq=1,prior=0,**kw):
        return self.c.command(ws,dict(seq=seq,speed=.1,steer=0,token='fresh',epoch=0,**kw),prior)
    def test_passive_tab_cannot_stop_driver(self):
        self.cmd(self.a)
        self.c.command(self.other,dict(seq=1,speed=0,steer=0),0)
        self.assertEqual(self.sent,[(.1,0)])
        self.assertEqual(self.cmd(self.other)[1],'another_driver')
    def test_stale_commands_not_replayed(self):
        self.cmd(self.a)
        self.assertEqual(self.cmd(self.a,prior=1)[1],'stale_command')
        self.b.leases['fresh']-=1
        self.assertEqual(self.cmd(self.a,seq=2,prior=1)[1],'expired_lease')
        self.assertEqual(len(self.sent),1)
    def test_emergency_invalidates_inflight_commands(self):
        self.cmd(self.a)
        self.c.command(self.other,dict(seq=1,speed=0,steer=0,emergency=True),0)
        self.assertEqual(self.sent[-1],(0,0))
        self.assertEqual(self.cmd(self.a,seq=2,prior=1)[1],'emergency_disarmed')
        self.assertFalse(self.b.leases)
    def test_control_is_independent_of_display_lock(self):
        self.b.lock=threading.Lock();self.b.lock.acquire()
        try:self.assertIsNone(self.cmd(self.a)[1])
        finally:self.b.lock.release()
    def test_map_transition_blocks_motion(self):
        self.b.drive_block=lambda:'map_transition'
        self.assertEqual(self.cmd(self.a)[1],'map_transition');self.assertFalse(self.sent)

if __name__=='__main__':unittest.main()
