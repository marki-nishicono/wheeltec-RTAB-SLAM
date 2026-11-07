"""Exercise vehicle kinematics without importing ROS or commanding hardware."""
import ast,math,pathlib,unittest
source=pathlib.Path(__file__).resolve().parents[1]/'scripts/control_guard.py'
tree=ast.parse(source.read_text())
function=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='ackermann')
namespace={'math':math}
exec(compile(ast.Module(body=[function],type_ignores=[]),str(source),'exec'),namespace)
ackermann=namespace['ackermann']

class Kinematics(unittest.TestCase):
    def test_stationary_cannot_spin(self):self.assertEqual(ackermann(0,.3,.143),(0,0))
    def test_reverse_yaw_reverses(self):
        self.assertAlmostEqual(ackermann(.08,.3,.143)[1],-ackermann(-.08,.3,.143)[1])
    def test_limits(self):
        v,w=ackermann(99,99,.143);self.assertEqual(v,.12);self.assertAlmostEqual(w,.12*math.tan(.35)/.143)
    def test_invalid(self):
        for args in [(float('nan'),0,.143),(1,float('inf'),.143),(1,0,0)]:
            with self.assertRaises(ValueError):ackermann(*args)

if __name__=='__main__':unittest.main()
