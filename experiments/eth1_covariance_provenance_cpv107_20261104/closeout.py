"""Read-only integrity audit; write only new CP107 supplemental receipts/seal."""
import datetime,hashlib,json,stat,subprocess
from pathlib import Path
H=Path(__file__).resolve().parent;E=H.parent;W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
P=W/'runs/eth1_prior_fix_live_cpv103_20261103/prepared'
def read(p):return json.loads(Path(p).read_text())
def sha(p):
    p=Path(p);assert stat.S_ISREG(p.stat().st_mode),p
    with p.open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def check(entries):
    for p,d in entries.items():assert sha(p)==d,p
    return len(entries)
def save(name,value):
    with (H/name).open('x') as f:json.dump(value,f,indent=2,allow_nan=False)
assert not (H/'complete.json').exists() and not (H/'closeout.json').exists()
counts={}
for name in ('analysis.json','insertions.json','verification.json'):
    r=read(H/name);assert r['passed'];counts[name]=check(r['input_sha256'])
protected=read(P/'protected_manifest.json');assert check(protected)==8120
old={}
for name,want in [('eth1_prior_fix_live_cpv103_20261103',62),
                  ('eth1_prior_fix_evaluation_cpv104_20261104',318),
                  ('eth1_fusion_diagnosis_cpv105_20261104',12),
                  ('eth1_covariance_freeze_cpv106_20261104',186),
                  ('eth1_split_imu_live_pair_cpv95_20261103_continuation1',426)]:
    r=read(E/name/'complete.json');assert r['passed'];old[name]=check(r['output_sha256']);assert old[name]==want
candidate=W/'runs/prior_anchor_integration_cpv102_20261103/candidate'
code=[W/'src/cbs/src/bpsam/bpsam.cpp', candidate/'VioBackend.cpp',candidate/'BpsamPersistent.inc',
    candidate/'include/cbs/bpsam/incremental_fixed_lag_bpsam_smoother.h',candidate/'OriginalChartPrior.h',
    E/'receiver_finalization_cpk19_20261021/Receiver.h',
    P/'transport_tools/live_receiver_protocol.py']
review={}
cp102=read(E/'prior_anchor_integration_cpv102_20261103/complete.json')['output_sha256']
for p in code:
    v=sha(p);p=str(p);expected=protected.get(p) or cp102.get(p)
    if expected is not None:assert v==expected,p
    review[p]=dict(sha256=v,pinned_in_CP103_or_CP102=expected is not None)
assert all(review[str(p)]['pinned_in_CP103_or_CP102'] for p in code[:5])
reports={}
for stem in ('eig2_marginal_handover_cpv67_20261031','glim_persistent_compat_cpv82_20261101',
             'eth1_kimera_early_covariance_cpv97_20261103','eth1_early_native_capture_cpv98_20261103'):
    path=E/stem/'REPORT.txt';reports[str(path)]=sha(path)
    if str(path) in protected:assert reports[str(path)]==protected[str(path)]
save('code_review.json',dict(passed=True,code=review,retained_prior_reports_sha256=reports,
    actual_CP103_loaded_backend_sha256=sha(P/'on/private_build/libkimera_vio.so'),
    finding='Current persistent lifecycle absorbs peer factors; LOCAL removes explicit tracked slots, not absorbed prior information. No per-event CP103 clean-history counterfactual recorded.',
    no_code_or_runtime_changes=True))
states=subprocess.check_output(['docker','inspect','-f','{{.Name}} {{.State.Status}}',
    'cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'],text=True).splitlines()
assert states==['/cbsms_ws_gpu exited','/ubuntu-jammy-humble-nvidia exited']
cp=E/'checkpoints/20261023_hesai_four_sequences/verify_checkpoint.py'
result=subprocess.run(['/usr/bin/python3','-B',str(cp),'--originals'],text=True,capture_output=True,check=True,timeout=180)
assert '667 saved files; originals=True' in result.stdout
now=datetime.datetime.now().astimezone().isoformat()
save('closeout.json',dict(passed=True,finished=now,input_checks=counts,protected_files=8120,
    preserved_terminal_outputs=old,containers=states,hesai_checkpoint=result.stdout.strip(),
    source_runs=0,receiver_replays=0,EVO_calls=0,builds=0,deletions=0,
    scope='Used inputs and sealed terminal outputs rehashed. CP106 large replay archives were not read/rehashed by this diagnostic; their manifest files were preserved.',
    heartbeat_unchanged=True))
outputs={str(p):sha(p) for p in sorted(H.iterdir()) if p.is_file()}
save('complete.json',dict(passed=True,task='CP-V107',finished=now,output_sha256=outputs,
    conclusion='Systematic native Kimera late covariance contraction confirmed; mixed-prior mechanism exists, exact contribution to this contraction unquantified.',
    saved_data_and_code_only=True,source_runs=0,receiver_replays=0,scientific_settings_changed=False))
print(json.dumps(dict(passed=True,inputs=counts,protected=8120,old=old,containers=states,
    checkpoint=result.stdout.strip(),outputs=len(outputs)),indent=2))
