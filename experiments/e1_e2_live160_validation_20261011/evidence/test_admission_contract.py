import copy
import unittest
from admission_contract import kimera_outcomes, verify_candidates

COV = ':'.join(str(i + 1.) for i in range(36))
LIFE = 'CBS_BPSAM_ACTIVE_WINDOW_LIFECYCLE_ROW,k,12.0,25.0,78,0,0,0'
TRANSPORT = 'CBS_BPSAM_SOURCE_TRANSPORT_ROW,G2K,gp21,gp23,x9,x10,'+COV+','+COV
MATRIX = 'CBS_ODOM_FACTOR_MATRIX_ROW,G2K,x9,x10,active_window_temporary_odom_applied,'+COV
DROP = 'CBS_BPSAM_ACTIVE_WINDOW_RECEIVER_EDGE_ROW,k,g,x9->x10,active_window_pre_residual_observed,pre_residual=3,threshold=2.0,action=drop_before_stage,total_active=0'

class ContractTests(unittest.TestCase):
    def setUp(self):
        self.c = dict(update=12, receiver=(9,10),sender=(21,23),cov=tuple(float(i+1) for i in range(36)),
                      raw=(0.,)*6,aligned=(0.1,)*6,times=(1.,2.,1.1,2.1,3.))
        self.wire = {(21,23):[dict(cov=self.c['cov'],mu=self.c['raw'],ti=1.,tj=2.)]}
    def run_case(self, lines, c=None, wire=None):
        return verify_candidates([c or self.c],kimera_outcomes(lines),wire or self.wire)
    def test_constructed(self):
        self.assertEqual(self.run_case([TRANSPORT,LIFE,MATRIX])[0]['outcome'],'constructed')
    def test_explicit_drop(self):
        self.assertEqual(self.run_case([TRANSPORT,LIFE,DROP])[0]['outcome'],'drop_before_stage')
    def test_post_stage_rollback_not_missing_construction(self):
        self.run_case([TRANSPORT,LIFE,MATRIX,'action=rollback_and_retry_without_staged_cbs'])
        with self.assertRaises(AssertionError): self.run_case([TRANSPORT,LIFE,'action=rollback_and_retry_without_staged_cbs'])
    def test_missing(self):
        with self.assertRaises(AssertionError): self.run_case([TRANSPORT,LIFE])
    def test_wrong_update(self):
        with self.assertRaises(AssertionError): self.run_case([TRANSPORT,LIFE.replace('12.0','13.0'),DROP])
    def test_unrelated_edge(self):
        with self.assertRaises(AssertionError): self.run_case([TRANSPORT,LIFE,DROP.replace('x9->x10','x8->x9')])
    def test_duplicate_guard(self):
        with self.assertRaises(AssertionError): self.run_case([TRANSPORT,LIFE,DROP,DROP])
    def test_conflicting_outcomes(self):
        with self.assertRaises(AssertionError): self.run_case([TRANSPORT,LIFE,MATRIX,DROP])
    def test_covariance_corruption(self):
        with self.assertRaises(AssertionError): self.run_case([TRANSPORT,LIFE,MATRIX.replace(',1.0:',',999.0:')])
    def test_rejected_wire_still_checked(self):
        c=copy.deepcopy(self.c);c['cov']=(99.,)*36
        with self.assertRaises(AssertionError):self.run_case([TRANSPORT,LIFE,DROP],c)
    def test_wrong_sender(self):
        with self.assertRaises(AssertionError):self.run_case([TRANSPORT.replace('gp21','gp20'),LIFE,DROP])
    def test_threshold_evidence(self):
        with self.assertRaises(AssertionError):self.run_case([TRANSPORT,LIFE,DROP.replace('pre_residual=3','pre_residual=1')])
    def test_timestamp_corruption(self):
        c=copy.deepcopy(self.c);c['times']=(1.,2.01,1.1,2.1,3.)
        with self.assertRaises(AssertionError):self.run_case([TRANSPORT,LIFE,DROP],c)
    def test_future_endpoint(self):
        c=copy.deepcopy(self.c);c['times']=(1.,2.,1.1,3.1,3.)
        with self.assertRaises(AssertionError):self.run_case([TRANSPORT,LIFE,DROP],c)
    def test_no_update(self):
        with self.assertRaises(AssertionError):self.run_case([TRANSPORT,DROP])

if __name__ == '__main__':unittest.main()
