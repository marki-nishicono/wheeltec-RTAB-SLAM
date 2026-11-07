"""Regression: HTTP responses and cached images must not imply a live base."""
import ast
import pathlib
import unittest

tree = ast.parse(pathlib.Path('scripts/edge_bridge.py').read_text())
cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'Bridge')
method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'drive_block')
class Clock:
    @staticmethod
    def monotonic(): return 100.
namespace = {'time': Clock}
exec(compile(ast.Module(body=[method], type_ignores=[]), '<health>', 'exec'), namespace)

class Health(unittest.TestCase):
    def setUp(self):
        self.sessions = None
        self.control_seen = dict(control=99.9, wheel=99.9, scan_raw=99.9)
        self.control_live = {'control': {'enabled': True, 'reason': 'deadman_timeout'}}
    def block(self): return namespace['drive_block'](self)
    def test_idle_is_drivable(self): self.assertIsNone(self.block())
    def test_stale_control_despite_live_http(self):
        self.control_seen['control'] = 50
        self.assertEqual(self.block(), 'control_offline')
    def test_wheel_disconnect(self):
        del self.control_seen['wheel']
        self.assertEqual(self.block(), 'wheel_offline')
    def test_scan_disconnect(self):
        self.control_seen['scan_raw'] = 99
        self.assertEqual(self.block(), 'scan_raw_offline')
    def test_calibrating(self):
        self.control_live['control']['reason'] = 'imu_calibrating_keep_stationary'
        self.assertEqual(self.block(), 'imu_calibrating_keep_stationary')
    def test_obstacle_does_not_lock_opposite_direction(self):
        self.control_live['control']['reason'] = 'front_obstacle_or_no_returns'
        self.assertIsNone(self.block())

if __name__ == '__main__': unittest.main()
