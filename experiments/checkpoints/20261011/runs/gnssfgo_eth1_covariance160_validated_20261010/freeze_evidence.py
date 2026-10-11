"""Protect reused native outputs and evaluation code before scoring."""
from pathlib import Path
import hashlib
import json
ROOT=Path(__file__).resolve().parent
OLD=ROOT.parent/'gnssfgo_eth1_covariance160_20261010'
PERF=ROOT.parent/'glim_cbs_diagnostic_budget_20261010'
paths=[OLD/'prepared/full160_off_source/odom_imu.txt',
       OLD/'prepared/full160_off_source/kimera_odometry.csv',
       OLD/'prepared/full160_off_source/source_coverage.json',
       OLD/'REPORT.txt',OLD/'failure_evidence.json',PERF/'REPORT.txt',PERF/'FINAL_VALIDATION.json',
       ROOT/'evaluate_full160.py',ROOT/'postvalidate.py',ROOT/'summarize_timing.py',
       ROOT.parent/'gnssfgo_eth1_active_window40_off_20261008/full_reference_evaluation_20261008/evaluate.py']
result={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
with (ROOT/'additional_protected_evidence.json').open('x') as f:json.dump(result,f,indent=2)
print('Protected',len(result),'additional retained outputs and validation/scoring scripts')
