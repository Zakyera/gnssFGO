"""Correct the identified log-routing gate using the ONE saved CP83 capture.

No estimator/receiver replay, container start, new capture or protected edit.
The original failed gate and original terminal controller evidence stay intact.
"""
import datetime,difflib,hashlib,json,os,subprocess,time
from pathlib import Path
H=Path(__file__).resolve().parent
P=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs')/H.name/'prepared'
V=Path('/home/yeranis/results')/H.name
R=H/'saved_validation_repair_1'
PY='/home/yeranis/.local/share/pipx/venvs/evo/bin/python'
def read(p):return json.loads(p.read_text())
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def save(p,v):
    with p.open('x') as f:json.dump(v,f,indent=2,allow_nan=False)
def write(p,t):
    with p.open('x') as f:f.write(t)
def call(args):return subprocess.check_output(args,text=True)
def containers():return call(['docker','inspect','-f','{{.Name}} {{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia']).splitlines()
def exited(receipt):
    p=Path('/proc')/str(receipt['pid'])/'stat'
    return not p.exists() or p.read_text().split()[21]!=receipt['proc_start']
def pins():
    frozen=read(P/'protected_manifest.json')
    for p,d in frozen.items():assert sha(p)==d,p
    return len(frozen)
assert read(H/'failed.json')=={'stage':'validate_on_candidate','error':"AssertionError(('validate_on_candidate', 1))"}
assert 'assert "injected_persistent" in log' in (H/'validate_on_candidate.log').read_text()
assert read(H/'closeout.json')['passed'] and read(H/'status.json')['state']=='failed'
assert exited(read(H/'start.json')) and all(exited(read(p)) for p in H.glob('*_process.json'))
assert read(H/'capture_on_receipt.json')['exit']==0
assert not (H/'validate_on_receiver_process.json').exists() and not (V/'on').exists()
S=P/'full_on_source';REC=P/'full_on_receiver';CTL=P/'full_on_control'
assert (S/'complete.marker').exists() and (REC/'complete.marker').exists()
assert (CTL/'complete.json').exists() and not (CTL/'failed.txt').exists()
assert not (S/'candidate_validation.json').exists()
assert read(S/'hesai_validation.json')['passed'] and read(S/'local_provenance.json')['passed']
before=containers();assert before==['/cbsms_ws_gpu exited','/ubuntu-jammy-humble-nvidia exited']
count=pins();R.mkdir(exist_ok=False)
originals={str(p):sha(p) for p in H.iterdir() if p.is_file() and p.name!='repair_saved_validation.py'}
save(R/'start.json',dict(started=datetime.datetime.now().astimezone().isoformat(),original_evidence=originals,
    original_controller_failure_preserved=True,source_captures=0,receiver_replays=0,protected_files=count))
def step(label,args):
    with (R/(label+'.log')).open('x') as log:
        job=subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT)
        save(R/(label+'_process.json'),dict(pid=job.pid,proc_start=Path(f'/proc/{job.pid}/stat').read_text().split()[21],command=args))
        code=job.wait()
    save(R/(label+'_receipt.json'),dict(exit=code,finished=time.time()))
    print(label,code,flush=True)
    assert code==0,(label,code)
try:
    original=(P/'validate_candidate.py').read_text()
    candidate=original.replace('R = Path(__file__).resolve().parent','R = Path('+repr(str(P))+')')
    marker='log = launcher_log + "\\n" + re.sub(r"\\x1b\\[[0-9;]*m", "", node_logs[0].read_text())'
    assert marker in candidate
    addition='\nglim_logs=list(S.glob("ros_logs/*/glim_rosnode-*-stdout.log"))\nassert len(glim_logs)==1,glim_logs\nlog += "\\n" + re.sub(r"\\x1b\\[[0-9;]*m", "", glim_logs[0].read_text())'
    candidate=candidate.replace(marker,marker+addition)
    candidate=candidate.replace('(S / "candidate_validation.json")','(Path('+repr(str(R)) +') / "candidate_validation.json")')
    write(R/'validate_candidate.py',candidate)
    write(R/'validator.diff',''.join(difflib.unified_diff(original.splitlines(True),candidate.splitlines(True),fromfile=str(P/'validate_candidate.py'),tofile=str(R/'validate_candidate.py'))))
    step('candidate',[PY,'-B',str(R/'validate_candidate.py'),'on'])
    # This receiver validation was never reached by the original controller.
    step('receiver',[PY,'-B',str(P/'validate_case.py'),'check','on'])
    assert read(R/'candidate_validation.json')['passed'] and read(V/'on/validation.json')['passed']
    save(R/'on_validated.json',dict(passed=True,supplemental=True,original_failure=read(H/'failed.json'),
        reason='GLIM positive injection assertion read only launcher/Kimera log; added exact GLIM stdout log',
        gates=[str(S/'hesai_validation.json'),str(S/'local_provenance.json'),str(R/'candidate_validation.json'),str(V/'on/validation.json')],
        capture_repeated=False,guards_relaxed=False))
    count=pins();assert containers()==before
    save(R/'closeout.json',dict(passed=True,containers=before,protected_files=count,
        hesai=call(['python3',str(H.parent/'checkpoints/20261023_hesai_four_sequences/verify_checkpoint.py'),'--originals']),
        original_owned_jobs_exited=True,no_new_estimator_process=True))
    save(R/'capture_complete.json',dict(passed=True,duration_s=60.0,original_capture=str(CTL/'complete.json'),
        saved_only_repair=True,fresh_source_runs=0,receiver_replays=0))
    write(R/'PROTOCOL.txt','SAVED-ONLY VALIDATION/REPORT REPAIR, CP83.\nOriginal capture protocol follows unchanged.\n'+(H/'PROTOCOL.txt').read_text())
    evaluation=(H/'evaluate.py').read_text()
    evaluation=evaluation.replace('H=Path(__file__).resolve().parent;E=H.parent','H=Path(__file__).resolve().parent;E=H.parent.parent')
    evaluation=evaluation.replace("P=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs')/H.name/'prepared'",'P=Path('+repr(str(P))+')')
    evaluation=evaluation.replace("V=Path('/home/yeranis/results')/H.name;O=H/'evaluation'",'V=Path('+repr(str(V))+ ");O=H/'evaluation'")
    evaluation=evaluation.replace("read(src/'candidate_validation.json')","read(H/'candidate_validation.json')")
    write(R/'evaluate.py',evaluation);write(R/'verify_report.py',(H/'verify_report.py').read_text())
    step('evaluate',[PY,'-B',str(R/'evaluate.py')])
    step('verify_report',[PY,'-B',str(R/'verify_report.py')])
    assert read(R/'report_verification.json')['passed']
    assert all(sha(p)==d for p,d in originals.items())
    count=pins();assert containers()==before
    assert all(exited(read(p)) for p in R.glob('*_process.json'))
    write(R/'REPORT.txt','CP83 requested60s live capture and corrected saved-only validation/EVO report PASS.\n'
          'Original controller failed solely because GLIM check omitted GLIM log; failed artifacts unchanged.\n'
          'No capture or receiver rerun, source/config/binary/covariance change, or guard relaxation.\n'
          'See RESULTS_REPORT.txt/results.json. Both receivers persistent; not full355s or a matched OFF/ON pair.\n'
          'Both containers stopped; owned processes exited;1989protected/Hesai667 pass.\n'
          'Kimera mixed-prior outgoing information remains NOT native-only; useraccepted.\n')
    save(R/'complete.json',dict(passed=True,finished=datetime.datetime.now().astimezone().isoformat(),
        saved_only_repair=True,original_controller_failure_preserved=True,source_captures=0,receiver_replays=0,
        protected_files=count,containers=before,output_sha256={str(p):sha(p) for p in R.rglob('*') if p.is_file()}))
    print('SAVED_ONLY_REPAIR_COMPLETE',str(R),flush=True)
except BaseException as exc:
    save(R/'failed.json',dict(error=repr(exc),source_retry_authorized=False))
    raise
