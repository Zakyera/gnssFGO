import subprocess
import unittest
from pathlib import Path
import numpy as np
from coverage_checks import check

class HarnessTests(unittest.TestCase):
    def test_actual_admission_guard(self):
        guard,=[s for s in (Path(__file__).parent/'source_runner.sh').read_text().splitlines()
                if s.startswith('[[ ( "${cbs_enable}"')]
        for mode in ['true','false','invalid']:
            for duration in ['8.0','10.0','40.0','160.0','346.0']:
                prefix=f'cbs_enable={mode}; cbs_direction=bidirectional; duration_sec={duration}; bag_start_sec=0.0; '
                r=subprocess.run(['bash','-c',prefix+guard],capture_output=True)
                self.assertEqual(r.returncode==0,mode!='invalid' and duration=='160.0')
    def test_complete(self):
        t=np.arange(.7,159.8,.1);p=np.zeros((len(t),3))
        self.assertGreater(check(t,p,160,0,159.85,'test')['span_s'],158)
    def test_truncated(self):
        t=np.arange(.7,120,.1)
        with self.assertRaises(AssertionError):check(t,np.zeros((len(t),3)),160,0,159.85,'test')
    def test_tail_missing(self):
        t=np.arange(.1,158.9,.1)
        with self.assertRaises(AssertionError):check(t,np.zeros((len(t),3)),160,0,159.85,'test')
    def test_disordered(self):
        t=np.arange(.7,159.8,.1);t[10]=t[9]
        with self.assertRaises(AssertionError):check(t,np.zeros((len(t),3)),160,0,159.85,'test')
    def test_startup_gap_reported(self):
        t=np.r_[.2,np.arange(3.4,159.9,.2)]
        r=check(t,np.zeros((len(t),3)),160,0,159.85,'test')
        self.assertAlmostEqual(r['maximum_pose_gap_s'],3.2)
    def test_clock_truncated(self):
        t=np.arange(.7,159.8,.1)
        with self.assertRaises(AssertionError):check(t,np.zeros((len(t),3)),160,0,158,'test')
    def test_late_hole(self):
        t=np.r_[np.arange(.7,40,.1),np.arange(42,159.8,.1)]
        with self.assertRaises(AssertionError):check(t,np.zeros((len(t),3)),160,0,159.85,'test')

if __name__=='__main__':unittest.main()
