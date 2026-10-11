import unittest
from lifecycle_contract import check_kimera
P='CBS_BPSAM_ACTIVE_WINDOW_RECEIVER_EDGE_ROW,k,g,'
def life(n):return 'CBS_BPSAM_ACTIVE_WINDOW_LIFECYCLE_ROW,k,1.0,25,78,0,'+str(n)+',0'
def tracked(edge,slot,total):return P+edge+',active_window_slot_tracked,slot='+str(slot)+',total_active='+str(total)
def remove(edge,slot,total,committed=False):return P+edge+(',active_window_remove_committed,' if committed else ',active_window_remove_scheduled,')+'slot='+str(slot)+',reason='+('active_residual_regression' if committed else 'update_remove_factor')+',slot_valid=true,total_active='+str(total)
def rollback(reason):return 'CBS_BPSAM_ACTIVE_WINDOW_RECEIVER_EDGE_ROW,k,-,-->-,active_window_rollback,reason='+reason
SG=P+'x3->x4,active_window_staged_residual_observed,action=rollback_and_retry_without_staged_cbs'
AG=P+'x2->x3,active_window_active_residual_observed,slot=12,slot_valid=true,action=rollback_remove_active_and_retry'
SR='CBS_BPSAM_ACTIVE_WINDOW_RECEIVER_EDGE_ROW,k,-,-->-,active_window_retry_local_update_without_staged_cbs,reason=staged_residual_regression,staged_count=1,preserved_active=2'
AR='CBS_BPSAM_ACTIVE_WINDOW_RECEIVER_EDGE_ROW,k,-,-->-,active_window_retry_local_update_removing_active_cbs,reason=active_residual_regression,staged_count=0,active_removal_count=1,removal_slot_count=1,committed_removals=1,preserved_active=1'
class LifecycleTests(unittest.TestCase):
    def fixture(self):
        return [life(0),tracked('x1->x2',11,1),life(1),tracked('x2->x3',12,2),life(2),
                remove('x1->x2',11,1),tracked('x3->x4',13,2),SG,rollback('staged_residual_regression'),SR,
                remove('x1->x2',11,1),AG,rollback('active_residual_regression'),remove('x2->x3',12,1,True),AR,
                remove('x1->x2',11,0),life(0)]
    def test_nested_rollback(self):
        r=check_kimera(self.fixture());self.assertEqual((r['tracked'],r['removed_by_update'],r['remaining']),(2,2,0))
        self.assertEqual(len(r['rollbacks']),2)
    def test_wrong_removal_slot(self):
        f=self.fixture();f[13]=remove('x2->x3',99,1,True)
        with self.assertRaises(AssertionError):check_kimera(f)
    def test_shared_active_slot(self):
        f=self.fixture();f[3]=tracked('x2->x3',11,2)
        with self.assertRaises(AssertionError):check_kimera(f)
    def test_missing_commit(self):
        f=self.fixture();f.pop(13)
        with self.assertRaises(AssertionError):check_kimera(f)
    def test_unknown_reason(self):
        f=self.fixture();f[7]=rollback('unknown')
        with self.assertRaises(AssertionError):check_kimera(f)
    def test_missing_guard(self):
        f=self.fixture();f.pop(11)
        with self.assertRaises(AssertionError):check_kimera(f)
    def test_wrong_count(self):
        f=self.fixture();f[14]=AR.replace('committed_removals=1','committed_removals=2')
        with self.assertRaises(AssertionError):check_kimera(f)
    def test_missing_recursive_restore(self):
        f=self.fixture();f.pop(12)
        with self.assertRaises((AssertionError,KeyError)):check_kimera(f)
    def test_stale_postsolve_slot(self):
        f=self.fixture();f[11]=AG.replace('slot=12','slot=99')
        with self.assertRaises(AssertionError):check_kimera(f)
if __name__=='__main__':unittest.main()
