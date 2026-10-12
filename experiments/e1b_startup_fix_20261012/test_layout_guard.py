"""Small deterministic binary-layout regression; no estimators or data."""
from pathlib import Path
import unittest
from layout_guard import allocation, require_size

WS = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
RUN = WS/'runs/e1b_startup_fix_20261012'
STALE = WS/'build/glim_ros/CMakeFiles/glim_ros.dir/src/glim_ros/glim_ros.cpp.o'
CURRENT = RUN/'fixed_release/glim_ros.o'
E1 = WS/'runs/glim_sync_compatible_batching_e1_20261011/private/glim_ros.o'


class LayoutTests(unittest.TestCase):
    def test_bad_allocator_rejected(self):
        with self.assertRaisesRegex(ValueError, 'allocator=3296, header=3344'):
            require_size(STALE, 3344)

    def test_rebuilt_allocator_accepted(self):
        require_size(CURRENT, 3344)

    def test_known_working_e1_allocator_accepted(self):
        require_size(E1, 3344)

    def test_header_probe(self):
        self.assertEqual((RUN/'layout/size.log').read_text().strip(),
                         'GlimCbsBridge_size=3344')

    def test_unknown_pattern_rejected(self):
        with self.assertRaises(ValueError):
            allocation('no unique constructor/allocation proof')


if __name__ == '__main__':
    unittest.main(verbosity=2)
