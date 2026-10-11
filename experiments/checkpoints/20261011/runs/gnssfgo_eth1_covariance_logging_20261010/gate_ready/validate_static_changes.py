#!/usr/bin/env python3
"""Prove this task changed only the GLIM diagnostic helper and its call sites."""
import hashlib
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parent
WS=ROOT.parents[2]
source=WS/'src/glim_ros1'
before=ROOT.parent/'prepared/before'
old=(before/'cbs_bridge.cpp').read_text()
new=(source/'src/glim_ros/cbs_bridge.cpp').read_text()
def remove_helper(text):
    start=text.index('void GlimCbsBridge::logReceiverLocalCompetition(')
    end=text.index('bool GlimCbsBridge::appendActiveWindowTemporaryFactor(',start)
    return text[:start]+text[end:]
normalized=remove_helper(new)
normalized=normalized.replace('#include <gtsam/linear/GaussianFactorGraph.h>\n','')
normalized=normalized.replace('matched, smoother, new_values, *new_factors, inserted_covariance,\n        factor);',
                              'matched, smoother, new_values, *new_factors, inserted_covariance);')
normalized=normalized.replace('  // Same no-op-by-default diagnostic as the persistent path, before insertion.\n'
    '  logReceiverLocalCompetition(\n'
    '      matched, smoother, new_values, *new_factors, covariance, factor);\n','')
assert normalized==remove_helper(old), 'Non-diagnostic implementation changed'
oldh=(before/'cbs_bridge.hpp').read_text()
newh=(source/'include/glim_ros/cbs_bridge.hpp').read_text()
assert newh.replace('const gtsam::Matrix6& inserted_covariance,\n'
                   '      const gtsam::NonlinearFactor::shared_ptr& incoming_factor);',
                   'const gtsam::Matrix6& inserted_covariance);')==oldh
start=new.index('void GlimCbsBridge::logReceiverLocalCompetition(')
assert new.index('receiver_local_competition_audit_max_samples_ == 0u',start)<new.index('smoother.calculateEstimate()',start)
result=dict(passed=True,unchanged_outside_diagnostic_helper_and_calls=True,
            disabled_audit_returns_before_factor_or_graph_access=True,
            source_cpp_sha256=hashlib.sha256(new.encode()).hexdigest(),
            source_hpp_sha256=hashlib.sha256(newh.encode()).hexdigest())
with (ROOT/'static_changes_validation.json').open('x') as f: json.dump(result,f,indent=2)
print(json.dumps(result,indent=2))
