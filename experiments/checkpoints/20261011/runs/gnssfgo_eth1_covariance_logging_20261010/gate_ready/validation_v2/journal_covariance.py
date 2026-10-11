"""Reuse the preparation's unchanged read-only journal covariance checks."""
import hashlib
import importlib.util
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parent.parent
assert json.loads((ROOT/'capture_review.json').read_text())['captured_data_complete']
tool=Path('/home/yeranis/repos/gnssFGO/experiments/covariance_interaction_eth1_160_preparation_20261010/inspect_capture.py')
spec=importlib.util.spec_from_file_location('prepared_inventory',tool)
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
journal=ROOT/'gate_on_receiver/events.csv'
before=hashlib.sha256(journal.read_bytes()).hexdigest()
out=module.journal_inventory(journal)
assert before==hashlib.sha256(journal.read_bytes()).hexdigest()
out.update(passed=True,journal_sha256=before,tool_sha256=hashlib.sha256(tool.read_bytes()).hexdigest(),
           covariance_health='Every transported covariance finite, symmetric, SPD; all stable identities and availability times checked.',
           limitation='Marginal source uncertainty is NOT calibrated accuracy. Last observed revisions can be right-censored.')
with (ROOT/'journal_covariance_validation.json').open('x') as f:json.dump(out,f,indent=2)
print(json.dumps({'passed':True,'event_counts':out['event_counts'],
    'sources':{s:{k:v for k,v in rows.items() if k in ['transported_ids','versions']}
               for s,rows in out['sources'].items()}},indent=2))
