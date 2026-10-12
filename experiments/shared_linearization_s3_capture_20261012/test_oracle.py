"""Independent validator must reject altered results and incomplete input."""
from pathlib import Path
import tempfile
import unittest
import numpy as np
from oracle import Reader,Oracle

ROOT=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/shared_linearization_s3_capture_20261012')
class Tests(unittest.TestCase):
    def fixture(self):
        o=Oracle();s=None;p=None
        reader=Reader(ROOT/'release_1/fixture.bin');self.addCleanup(reader.f.close)
        for tag,r in reader.records():
            if tag=='S':s=r
            elif tag=='C':o.commit(s,r)
            elif tag=='P':p=r
            elif tag=='Q':return o,p,r
        self.fail('no query')
    def test_reject_corrupted_covariance(self):
        o,p,q=self.fixture();q['cov']*=1.01
        with self.assertRaises(AssertionError):o.query(p,q)
    def test_reject_corrupted_candidate_information(self):
        o,p,q=self.fixture();p['factors'][0][2][0,0]*=1.1
        with self.assertRaises(AssertionError):o.query(p,q)
    def test_reject_dropped_cross_covariance(self):
        o,p,q=self.fixture();q['cov'][:6,6:]=0;q['cov'][6:,:6]=0
        with self.assertRaises(AssertionError):o.query(p,q)
    def test_truncated_record(self):
        with tempfile.TemporaryDirectory(prefix='s3-oracle-') as tmp:
            p=Path(tmp)/'partial.bin';p.write_bytes((ROOT/'release_1/fixture.bin').read_bytes()[:50])
            reader=Reader(p);self.addCleanup(reader.f.close)
            with self.assertRaises(EOFError):list(reader.records())
    def test_encoded_values_and_dense_blocks(self):
        reader=Reader(ROOT/'release_1/fixture.bin');self.addCleanup(reader.f.close)
        records=list(reader.records());s=records[0][1]
        self.assertEqual(s['epoch'],1);self.assertEqual(len(s['refs']),1)
        f=next(iter(s['refs'].values()))[2]
        np.testing.assert_array_equal(f[2][:6,:6],np.eye(6)*(1/.3)**2)
        np.testing.assert_array_equal(f[2][6],0)
        self.assertEqual(next(iter(s['theta'].values()))[2].shape,(4,4))

if __name__=='__main__':unittest.main()
