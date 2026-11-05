"""Verify completed controls and restore-state evidence; never launch work."""
import hashlib,json,subprocess,time
from pathlib import Path
H=Path(__file__).resolve().parent
R=Path('/home/yeranis/results/gnssfgo_leica2_single_source_controls_cpv25_20261027')
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
assert not (H/'closeout.json').exists()
done=json.loads((H/'replay_complete.json').read_text());assert done['all_pass']
assert (H/'results.json').exists() and (H/'REPORT.txt').exists()
pins={}
for filename in ('input_pins.json','compatibility_pins.json','scoring_pins.json'):
    for p,d in json.loads((H/filename).read_text()).items():
        if p in pins:assert pins[p]==d
        pins[p]=d
for p,d in pins.items():assert sha(p)==d,p
prepared=json.loads((H/'prepared.json').read_text())
cases={}
for case,info in prepared['cases'].items():
    assert sha(R/(case+'_events.csv'))==info['filtered_sha256']
    assert sha(R/(case+'_original_lines.json'))==info['mapping_sha256']
    check=json.loads((H/(case+'_validation.json')).read_text());assert check['pass_']
    assert json.loads((H/(case+'_finish.json')).read_text())['returncode']==0
    for n,d in check['files_sha256'].items():assert sha(R/case/n)==d
    cases[case]=dict(summary=check['summary'],payload_action_sha256=check['retained_source_payload_action_sha256'])
states=json.loads(subprocess.check_output(['docker','inspect','ubuntu-jammy-humble-nvidia','cbsms_ws_gpu']))
containers={r['Name']:{k:r['State'][k] for k in ('Status','Running','Pid')} for r in states}
assert all(not v['Running'] and v['Pid']==0 for v in containers.values())
owned=[]
for p in Path('/proc').glob('[0-9]*/cmdline'):
    try:args=p.read_bytes().split(b'\0')
    except (OSError,PermissionError):continue
    if any(str(R/(case+'_events.csv')).encode() in args for case in prepared['cases']):owned.append(str(p.parent))
    if str(H/'run_compat.py').encode() in args:owned.append(str(p.parent))
assert not owned,owned
command=['python3',str(H.parent/'checkpoints/20261023_hesai_four_sequences/verify_checkpoint.py'),'--originals']
checkpoint=subprocess.run(command,text=True,capture_output=True);assert checkpoint.returncode==0,checkpoint.stderr
artifacts={str(p):sha(p) for p in H.iterdir() if p.is_file()}
result={'pass_':True,'time':time.time(),'unique_host_pinned_inputs':len(pins),
        'runtime_pins_verified_inside_container_after_replays':done['runtime_pins'],
        'cases':cases,'containers':containers,'owned_jobs_remaining':owned,
        'hesai_checkpoint':dict(command=command,returncode=checkpoint.returncode,stdout=checkpoint.stdout.strip()),
        'artifact_hashes':artifacts,'source_runs':0,'new_builds':0,'receiver_replays':2,'promoted':False}
with (H/'closeout.json').open('x') as f:json.dump(result,f,indent=2)
print(json.dumps({k:v for k,v in result.items() if k not in ('cases','artifact_hashes')},indent=2))
