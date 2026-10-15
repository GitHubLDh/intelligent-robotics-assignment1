"""Geometry sanity checks for 19cm low walls; not a Webots runtime test."""
from pathlib import Path
import re
import math
import unittest

ROOT = Path(__file__).resolve().parents[1]

class MountTests(unittest.TestCase):
    def test_lidar_below_low_wall_top(self):
        txt = (ROOT / 'protos/SimpleDiffRobot.proto').read_text()
        lidar = txt.split('      Lidar {',1)[1].split('      }',1)[0]
        dz = float(re.search(r'translation\s+0\s+0\s+([.\d]+)',lidar).group(1))
        vfov=float(re.search(r'verticalFieldOfView\s+([.\d]+)',lidar).group(1))
        height=.085+dz
        self.assertLess(height,.19)
        self.assertGreater(height, .125)
        # The lower scan layer clears the chassis top at its 11.5cm edge.
        self.assertGreater(height-.115*math.tan(vfov/2),.125)
        # And does not hit the floor within the full 2.5m sensor range.
        self.assertGreater(height-2.5*math.tan(vfov/2),0)
    def test_low_wall_diagnostic_has_correct_height_and_clearance(self):
        txt=(ROOT/'worlds/diagnostic_lidar_low_wall_close.wbt').read_text()
        self.assertIn('size 0.12 1.2 0.19',txt)
        self.assertIn('translation 0.46 0 0.095',txt)
        self.assertIn('name "R1"',txt)

if __name__ == '__main__': unittest.main()
