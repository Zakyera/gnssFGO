"""Read-only final integrity / ownership checks, followed by one receipt."""
import hashlib,json,subprocess,time
from pathlib import Path
H=Path(__file__).resolve().parent
R=Path('/home/yeranis/results/gnssfgo_leica2_incremental_full_cpv23_20261027')
P=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_leica2_velodyne_full_cpv4_20261024/prepared')
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
assert not (H/'closeout.json').exists()
pins={};counts={}
for file,key in [('input_pins.json',None),('scoring_pins.json',None),('reconstruction_checks.json','input_sha256'),('scene_build_checks.json','input_sha256')]:
    values=json.loads((H/file).read_text());values=values if key is None else values[key]
    for p,digest in values.items():
        if p in pins:assert pins[p]==digest,p
        pins[p]=digest
    counts[file]=len(values)
for p,digest in pins.items():assert sha(p)==digest,p
assert sha(R/'live_publications')==json.loads((H/'binary.json').read_text())['sha256']
for mode in ('off','on'):
    check=json.loads((H/f'{mode}_parity.json').read_text());assert check['pass_']
    for name,digest in check['byte_identical_outputs'].items():
        assert sha(R/mode/name)==sha(P/f'full_{mode}_receiver/receiver'/name)==digest
    rc=json.loads((H/'reconstruction_checks.json').read_text())['cases'][mode]
    assert sha(R/mode/'chained_trajectory.tum')==rc['chained_sha256']
assert sha(H/'leica2_incremental_comparison.rrd')==json.loads((H/'scene_recording_checks.json').read_text())['rrd_sha256']
states=json.loads(subprocess.check_output(['docker','inspect','ubuntu-jammy-humble-nvidia','cbsms_ws_gpu']))
container_states={x['Name']:{k:x['State'][k] for k in ('Status','Running','Pid')} for x in states}
assert all(not x['Running'] and x['Pid']==0 for x in container_states.values())
owned=[]
for p in Path('/proc').glob('[0-9]*/cmdline'):
    try:args=p.read_bytes().split(b'\0')
    except (OSError,PermissionError):continue
    if str(R/'live_publications').encode() in args or str(H/'run.py').encode() in args:owned.append(str(p.parent))
assert not owned,owned
cmd=['python3',str(H.parent/'checkpoints/20261023_hesai_four_sequences/verify_checkpoint.py'),'--originals']
checkpoint=subprocess.run(cmd,text=True,capture_output=True);assert checkpoint.returncode==0,checkpoint.stderr
viewer=json.loads((H/'viewer_launch.json').read_text())
cmdline=Path(f'/proc/{viewer["pid"]}/cmdline').read_bytes().split(b'\0')
assert str(H/'leica2_incremental_comparison.rrd').encode() in cmdline
artifacts={str(p):sha(p) for p in sorted(H.iterdir()) if p.is_file()}
artifacts.update({str(R/f'{m}_publications.csv'):sha(R/f'{m}_publications.csv') for m in ('off','on')})
receipt={'pass_':True,'time':time.time(),'unique_pinned_inputs':len(pins),'manifest_counts':counts,
    'post_replay_container_dependency_verification':json.loads((H/'replay_complete.json').read_text()),
    'container_states':container_states,'owned_replay_processes':owned,'viewer_pid':viewer['pid'],
    'hesai_checkpoint':{'command':cmd,'stdout':checkpoint.stdout.strip(),'returncode':checkpoint.returncode},
    'final_artifact_sha256':artifacts,'not_promoted':True,'source_reruns':0}
with (H/'closeout.json').open('x') as f:json.dump(receipt,f,indent=2)
print(json.dumps({k:v for k,v in receipt.items() if k!='final_artifact_sha256'},indent=2))
