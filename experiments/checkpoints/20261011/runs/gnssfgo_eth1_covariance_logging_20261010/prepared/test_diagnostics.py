import unittest
import numpy as np
from validate_diagnostics import check_factor, matrix

class DiagnosticsTests(unittest.TestCase):
    def test_dense_cross_terms_and_gradient(self):
        rng=np.random.default_rng(4); a=rng.normal(size=(6,6))
        cov=a@a.T+.3*np.eye(6); J=rng.normal(size=(6,12)); r=rng.normal(size=(6,1))
        H=J.T@np.linalg.solve(cov,J); g=J.T@np.linalg.solve(cov,r)
        c=.5*float((r.T@np.linalg.solve(cov,r)).item())
        check_factor(cov,r,J[:,:6],J[:,6:],H,g,c)
        with self.assertRaises(AssertionError):
            check_factor(cov,r,J[:,:6],J[:,6:],H,-g,c)
    def test_bad_covariance_rejected(self):
        with self.assertRaises(AssertionError):
            check_factor(-np.eye(6),np.zeros((6,1)),np.eye(6),np.eye(6),np.eye(12),np.zeros((12,1)),0)
    def test_row_major_tokens(self):
        np.testing.assert_array_equal(matrix('1:2:3;4:5:6'),[[1,2,3],[4,5,6]])

if __name__=='__main__':unittest.main()
