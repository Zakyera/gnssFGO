"""Retain completed CP56 diagnostic, including its unresolved strict failure."""
import collections,datetime,hashlib,json
from pathlib import Path
H=Path(__file__).resolve().parent
K=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs')/H.name
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
v=json.loads((H/'verification.json').read_text());assert v['preservation_passed'] and not v['strict_gauge_passed']
a=json.loads((H/'analysis.json').read_text());d=json.loads((H/'independent_dlt.json').read_text());assert a['completed'] and d['passed']
rows={r['slot']:r for line in (K/'gauge_decomposition/results/gauge.jsonl').open() if (r:=json.loads(line))['phase']=='ordinary_theta' and r['transform']=='identity'}
lost=[rows[r['slot']] for r in a['initially_valid_final_outliers']]
full=sum(all(c['z'][1] is not None for c in r['cameras']) for r in lost)
mono=sum(all(c['z'][1] is None for c in r['cameras']) for r in lost)
assert (len(lost),full,mono)==(27,16,11)
with (H/'support_detail_checks.json').open('x') as f:json.dump(dict(lost_valid_to_outlier=27,full_stereo_tracks=full,left_only_tracks=mono,report_numbers_verified=True),f,indent=2)
pins={str(p):sha(p) for root in (H,K) for p in root.rglob('*') if p.is_file() and p.name not in ('retained_manifest.json','complete.json')}
with (H/'retained_manifest.json').open('x') as f:json.dump(pins,f,indent=2)
complete=dict(completed=True,strict_world_origin_invariance_passed=False,preservation_passed=True,root_cause_proved=False,production_fix=False,source_replays=0,graph_optimizer_calls=0,covariance_weighting_changes=0,containers=v['containers'],owned_jobs_exited=True,hesai=v['hesai'],protected_inputs=v['protected_files'],retained_files=len(pins),retained_manifest_sha256=sha(H/'retained_manifest.json'),report_sha256=sha(H/'REPORT.txt'),time=datetime.datetime.now().astimezone().isoformat())
with (H/'complete.json').open('x') as f:json.dump(complete,f,indent=2)
print(json.dumps(complete,indent=2))
