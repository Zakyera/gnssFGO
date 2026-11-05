"""Final reporting receipt. Does not execute any experiment or scorer."""
import csv,datetime,hashlib,json,math,subprocess
from pathlib import Path
H=Path(__file__).resolve().parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
K=W/'runs'/H.name;D=K/'attempt2'
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def states(p):
    with p.open() as f:return {r[0]:list(map(float,r[1:])) for r in csv.reader(f)}
v=json.loads((H/'verification.json').read_text());assert v['passed'] and not v['provenance']['live_gate_passed']
assert (K/'attempt1/failed.json').is_file() and json.loads((K/'attempt1/release_compile_receipt.json').read_text())['exit']!=0
failed_inputs=json.loads((K/'attempt1/inputs.json').read_text())
assert sha(K/'attempt1/Handover.cpp')==failed_inputs[str(K/'Handover.cpp')]
pins=json.loads((D/'inputs.json').read_text())
for path,h in pins.items():assert sha(path)==h,path
containers=subprocess.check_output(['docker','inspect','-f','{{.Name}} {{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'],text=True)
assert containers==v['containers']
for attempt in ('attempt1','attempt2'):
    for receipt in [K/attempt/'start.json',*(K/attempt).glob('*_process.json')]:
        pid=json.loads(receipt.read_text())['pid'];assert not Path('/proc',str(pid)).exists(),(receipt,pid)
previous=states(W/'runs/eig2_retirement_capture_cpv66_20261031/causal/results/update_250/previous.csv')['x249']
revisions={}
for arm in ('A','B'):
    x=states(D/'results'/f'{arm}_survivors.csv')['x249']
    revisions[arm]=math.sqrt(sum((a-b)**2 for a,b in zip(x[:3],previous[:3])))
assert abs(revisions['A']-.335968909)<1e-6 and abs(revisions['B']-.062327801)<1e-6
manifest={str(p):sha(p) for p in K.rglob('*') if p.is_file()}
manifest.update({str(p):sha(p) for p in H.iterdir() if p.is_file()})
with (H/'artifact_sha256.json').open('x') as f:json.dump(manifest,f,indent=2)
result=dict(status='COMPLETE_SAVED_DIAGNOSTIC_LIVE_PROVENANCE_GATE_FAILED',
            diagnostic_verified=True,live_candidate_admitted=False,
            finished=datetime.datetime.now().astimezone().isoformat(),
            post_marginalization_previous_pose_revision_m=revisions,
            sources_launched=0,forward_continuation=False,EVO_scoring=False,
            production_changed=False,weights_changed=False,containers=containers,
            owned_jobs_exited=True,protected_inputs=len(pins),
            report_sha256=sha(H/'REPORT.txt'),verification_sha256=sha(H/'verification.json'),
            artifact_files=len(manifest),preserved_build_failure=True,
            conclusion='Single-boundary handover preserves the retained solution, but naive retention leaks absorbed peer information into the current LOCAL extraction. Conditional live run not launched.')
with (H/'complete.json').open('x') as f:json.dump(result,f,indent=2)
print(json.dumps(result,indent=2))
