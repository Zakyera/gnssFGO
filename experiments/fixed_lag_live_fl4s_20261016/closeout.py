"""Read-only final verification plus one fresh compact receipt; no estimators."""
from pathlib import Path
import hashlib, json, subprocess

H=Path(__file__).resolve().parent
R=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_fixed_lag_live30_off_on_fl4s2_20261016/prepared')
V=Path('/home/yeranis/results/gnssfgo_fixed_lag_live_fl4s_20261016')
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
protected=json.loads((R/'protected_manifest.json').read_text())
for p,want in protected.items():assert sha(p)==want,p
def tree(p):return {str(f.relative_to(p)):sha(f) for f in p.rglob('*') if f.is_file()}
for name in ('glim_config','kimera_profile'):
    assert tree(R/'pilot30_off_source'/name)==tree(R/'pilot30_on_source'/name)
metrics=json.loads((R/'evaluation_full/summary.json').read_text())
matched=json.loads((R/'evaluation_full/belief_covariance_matched.json').read_text())
assert sum(v['matched_edges'] for v in matched.values())==421
receipts={}
for mode in ('off','on'):
    for p in (R/f'pilot30_{mode}_source/candidate_validation.json',
              V/mode/'validation.json', V/mode/'repeat_validation.json'):
        assert json.loads(p.read_text())['passed'];receipts[str(p)]=sha(p)
    assert json.loads((V/mode/'validation.json').read_text())['summary']['unsupported_end']==0
    assert not (R/f'pilot30_{mode}_control/failed.txt').exists()
    assert (R/f'pilot30_{mode}_control/complete.json').exists()
    for output in ('native_glim','native_kimera','fused_online','fused_finalized'):
        result=metrics['results'][mode+'_'+output]
        host=result['path'].replace('/workspace/cbs_gtsam4.3','/home/yeranis/repos/V4RL/cbs_gtsam4.3')
        assert sha(host)==result['sha256']
containers=subprocess.check_output(['docker','inspect','--format','{{.Name}} {{.State.Status}}',
                                    'cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'],text=True).strip().splitlines()
assert containers==['/cbsms_ws_gpu exited','/ubuntu-jammy-humble-nvidia exited']
head=subprocess.check_output(['git','-C',str(H.parents[1]),'rev-parse','HEAD'],text=True).strip()
assert head=='67c00392cd640a432bda9b67afcb57be4face381'
out=dict(passed=True,protected_unchanged=len(protected),matched_profiles_byte_identical=True,
         containers=containers,validation_hashes=receipts,head=head,commit_created=False,
         report_sha256=sha(H/'REPORT.txt'),evaluation_sha256=sha(R/'evaluation_full/summary.json'),
         matched_covariance_sha256=sha(R/'evaluation_full/belief_covariance_matched.json'))
with (H/'closeout.json').open('x') as f:json.dump(out,f,indent=2)
print(json.dumps(out,indent=2))
