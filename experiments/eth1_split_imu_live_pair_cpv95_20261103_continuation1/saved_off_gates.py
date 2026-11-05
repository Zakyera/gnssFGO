"""Once-only saved-data validation. Never starts ROS, an estimator, or a receiver."""
import datetime,json,os,subprocess
from pathlib import Path
from paths import H,P,PY,V
def save(name,value):
    with (H/name).open('x') as f:json.dump(value,f,indent=2)
assert json.loads((H/'validator_test.json').read_text())['passed']
assert (P/'full_off_control/complete.json').is_file()
assert not any(P.glob('full_on_*'))
save('saved_off_gates_start.json',dict(pid=os.getpid(),proc_start=Path('/proc/self/stat').read_text().split()[21],
     started=datetime.datetime.now().astimezone().isoformat(),source_runs=0,receiver_replays=0))
for label,script,args in [('source','validate_source.py',['off']),
                          ('payload_route','validate_local.py',['off']),
                          ('receiver','validate_case.py',['check','off']),
                          ('frame','frame_off.py',['off'])]:
    name='validate_off_'+label;command=[PY,'-B',str(H/script),*args]
    with (H/(name+'.log')).open('x') as log:
        job=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT)
        save(name+'_process.json',dict(pid=job.pid,args=command,proc_start=Path(f'/proc/{job.pid}/stat').read_text().split()[21]))
        code=job.wait()
    save(name+'_receipt.json',dict(exit=code,finished=datetime.datetime.now().astimezone().isoformat()))
    assert code==0,(name,'Saved integrity gate failed: no ON launch')
    print('PASS '+name,flush=True)
gates=[H/'off_candidate_validation.json',H/'off_local_provenance.json',V/'off/validation.json',H/'frame_off.json']
assert all(json.loads(p.read_text())['passed'] for p in gates)
save('off_validated.json',dict(passed=True,saved_capture=str(P/'full_off_source'),
    gates=list(map(str,gates)),source_runs=0,receiver_replays=0,finished=datetime.datetime.now().astimezone().isoformat()))
