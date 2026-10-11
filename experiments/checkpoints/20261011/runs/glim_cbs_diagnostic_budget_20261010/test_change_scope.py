"""Structural regression check against the exact retained pre-edit wrapper."""
from pathlib import Path
import json
import re
import unittest

ROOT=Path(__file__).resolve().parent
CPP=ROOT.parents[1]/'src/glim_ros1/src/glim_ros/cbs_bridge.cpp'

def methods(text):
    found=list(re.finditer(r'(?m)^[\w :<>*&]*GlimCbsBridge::(~?\w+)\(',text))
    result={}
    for i,m in enumerate(found):
        result[m[1]]=text[m.start():found[i+1].start() if i+1<len(found) else len(text)]
    return result

class Scope(unittest.TestCase):
    def test_all_other_bridge_methods_unchanged(self):
        before=methods((ROOT/'before/cbs_bridge.cpp').read_text())
        after=methods(CPP.read_text())
        self.assertGreater(len(before),30)
        self.assertEqual(before.keys(),after.keys())
        changed={k for k in before if before[k]!=after[k]}
        self.assertEqual(changed,{'configure','onSmootherUpdateFinish','logActiveWindowFactorDiagnostics'})
        for required in ['updateSidecar','publishOutgoingBeliefs','onSmootherUpdate']:
            self.assertIn(required,before)
            self.assertEqual(before[required],after[required])

    def test_gate_precedes_all_expensive_operations(self):
        method=methods(CPP.read_text())['logActiveWindowFactorDiagnostics']
        gate=method.index('if (!active_factor_diagnostics_budget_.shouldSample())')
        self.assertIn('return;',method[gate:method.index('gtsam::Values estimate;')])
        for op in ['smoother.calculateEstimate()', 'factorErrorOrNan(', 'factorPoseHessianFrobeniusOrNan(']:
            self.assertGreater(method.index(op),gate)
        self.assertLess(method.index('++item.second.active_update_count;'),gate)
        self.assertEqual(method.count('active_update_count;'),1)

if __name__=='__main__':unittest.main()
