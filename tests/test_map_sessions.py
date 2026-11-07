import hashlib
import pathlib
import sqlite3
import sys
import tempfile
import unittest
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]/'scripts'))
from map_sessions import MapStore,MatchGate,checked_label


class SavedMaps(unittest.TestCase):
    def test_working_copy_never_mutates_saved_map(self):
        with tempfile.TemporaryDirectory() as directory:
            root=pathlib.Path(directory);source=root/'source.db'
            with sqlite3.connect(source) as db:
                db.execute('CREATE TABLE Node(id INTEGER PRIMARY KEY)');db.execute('INSERT INTO Node VALUES(1)')
            store=MapStore(root/'maps')
            saved=store.save(source,'实验室 / 一号地图',{'nodes':[]},{})
            original=store.saved/(saved['id']+'.db');before=hashlib.sha256(original.read_bytes()).hexdigest()
            metadata,working,ids,view=store.copy_for_resume(saved['id'])
            with sqlite3.connect(working) as db:db.execute('INSERT INTO Node VALUES(2)')
            self.assertEqual(hashlib.sha256(original.read_bytes()).hexdigest(),before)
            self.assertEqual(ids,{1});self.assertEqual(store.list()[0]['label'],'实验室 / 一号地图')
            for value in ['../../etc/passwd','a'*31,'G'*32,None]:
                with self.assertRaises(ValueError):store.manifest(value)
    def test_bad_database_does_not_create_saved_entry(self):
        with tempfile.TemporaryDirectory() as directory:
            root=pathlib.Path(directory);bad=root/'bad';bad.write_text('invalid')
            store=MapStore(root/'maps')
            with self.assertRaises(sqlite3.DatabaseError):store.save(bad,'bad',{}, {})
            self.assertEqual(store.list(),[])
    def test_delete_snapshot_preserves_active_working_copy(self):
        with tempfile.TemporaryDirectory() as directory:
            root=pathlib.Path(directory);source=root/'source.db'
            with sqlite3.connect(source) as db:
                db.execute('CREATE TABLE Node(id INTEGER PRIMARY KEY)');db.execute('INSERT INTO Node VALUES(1)')
            store=MapStore(root/'maps');saved=store.save(source,'delete test',{}, {})
            _,working,_,_=store.copy_for_resume(saved['id'])
            before=working.read_bytes();store.delete(saved['id'])
            self.assertEqual(store.list(),[]);self.assertEqual(working.read_bytes(),before)
            self.assertEqual(list(store.saved.iterdir()),[])
            with self.assertRaises(ValueError):store.delete('../../outside')
    def test_label_limits(self):
        with self.assertRaises(ValueError):checked_label('x'*81)
        with self.assertRaises(ValueError):checked_label('x\ny')


class Relocalization(unittest.TestCase):
    def test_two_fresh_confirmed_matches_required(self):
        g=MatchGate({1,2},100)
        for t in [101,102]:
            g.info(t,1,0);g.pose(t,[1,2,.1],[.01,.01,.01])
            if t==101:self.assertIsNone(g.accepted)
        self.assertEqual(g.accepted['node_id'],1)
    def test_wrong_map_stale_and_invalid_covariance_are_rejected(self):
        g=MatchGate({1},100)
        for t in [99,100,101,102]:
            g.info(t,999,0);g.pose(t,[0,0,0],[.01]*3)
        self.assertIsNone(g.accepted)
        for t in [103,104]:
            g.info(t,1,0);g.pose(t,[0,0,0],[9999]*3)
        self.assertIsNone(g.accepted)
    def test_duplicate_pose_cannot_count_twice(self):
        g=MatchGate({1},100);g.info(101,0,1)
        for _ in range(3):g.pose(101,[0,0,0],[.01]*3)
        self.assertEqual(g.count,1);self.assertIsNone(g.accepted)
    def test_unstable_match_does_not_pass(self):
        g=MatchGate({1},100)
        for t in [101,102,103]:
            g.info(t,1,0);g.pose(t,[t,0,0],[.01]*3)
        self.assertIsNone(g.accepted)

if __name__=='__main__':unittest.main()
