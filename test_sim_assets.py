import json
import math
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

from build_sim import STATIONS, build
from shelf_core import Registry


class SimAssetTests(unittest.TestCase):
    def test_generated_registry_geometry_and_map(self):
        # Placeholder grids test serialization/geometry, NOT valid ArUco patterns.
        grids = {i:[[0]*6 for _ in range(6)] for i in range(1,5)}
        with tempfile.TemporaryDirectory() as folder:
            root = build(grids,folder)
            world = ET.parse(root/'shelf_demo.world').getroot()
            models = world.findall('world/model')
            self.assertEqual(len(models),8)
            registry = Registry.load(root/'locations.sim.json')
            self.assertEqual(registry.data['map_id'],'gazebo_shelf_demo_v1')
            for key,_,x,y,yaw in STATIONS:
                goal = registry.lookup(key,'gazebo_shelf_demo_v1')
                self.assertEqual(goal['status'],'ready')
                pose = goal['approach_pose']
                self.assertAlmostEqual(math.hypot(x-pose['x'],y-pose['y']),0.8,places=3)
                heading = math.atan2(y-pose['y'],x-pose['x'])
                self.assertAlmostEqual(math.cos(heading-yaw),1,places=5)
            pgm = (root/'shelf_demo.pgm').read_bytes()
            self.assertEqual(len(pgm.split(b'\n',3)[3]),168*168)
            self.assertEqual(set(pgm.split(b'\n',3)[3]),{0,254})
            info = json.loads((root/'BUILD_INFO.json').read_text())
            self.assertTrue(info['simulation_only'])
            for model in models:
                self.assertEqual(model.findtext('static'),'true')


if __name__ == '__main__':
    unittest.main(verbosity=2)
