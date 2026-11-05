"""Read-only historical integrity checks plus new CP57 closeout artifacts."""
import datetime
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess

H=Path(__file__).resolve().parent
OLD=H.parent/'eig2_visual_support_cpv56_20261030'


def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()


def save(name,data):
    with (H/name).open('x') as f:json.dump(data,f,indent=2)


def main():
    pins={}
    for file in [OLD/'measure_gate_failure/protected_manifest.json',OLD/'gauge_decomposition/pins.json',
                 OLD/'retained_manifest.json',H/'inputs.json',H/'attempt_inner1/inputs.json',
                 H/'attempt_inner1/repair_pins.json']:
        data=json.loads(file.read_text())
        for p,h in data.items():
            if p in pins:assert pins[p]==h,p
            pins[p]=h
    for name in ['findings.json','boundary_analysis.json','convex_candidates.json']:
        for p,h in json.loads((H/name).read_text())['inputs'].items():
            if p in pins:assert pins[p]==h,p
            pins[p]=h
    for i,(p,h) in enumerate(pins.items()):
        assert sha(p)==h,p
        if i%2000==0:print('Verified',i,'of',len(pins),'protected/input artifacts',flush=True)
    findings=json.loads((H/'findings.json').read_text())
    cert=json.loads((H/'certificate_verification.json').read_text())
    assert findings['completed'] and findings['no_unresolved_geometry'] and cert['passed']
    assert cert['findings_sha256']==sha(H/'findings.json')
    assert json.loads((H/'attempt_inner1/selftest.json').read_text())['passed']
    containers=subprocess.check_output(['docker','inspect','-f','{{.Name}} {{.State.Status}}',
                                       'cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'],text=True)
    assert containers=='/cbsms_ws_gpu exited\n/ubuntu-jammy-humble-nvidia exited\n'
    # Check real host process arguments, excluding this verifier and shell/tool
    # wrappers. Only CP57 Python executables are owned jobs of this diagnostic.
    process=subprocess.check_output(['ps','-eo','pid=,comm=,args='],text=True)
    owned=[]
    for line in process.splitlines():
        fields=line.split(None,2)
        if len(fields)==3 and fields[1].startswith('python') and H.name in fields[2] and int(fields[0])!=os.getpid():owned.append(line)
    assert not owned,owned
    hesai=subprocess.check_output(['python3',str(H.parent/'checkpoints/20261023_hesai_four_sequences/verify_checkpoint.py'),'--originals'],text=True)
    assert 'PASS: 4 cases, 667 saved files; originals=True' in hesai
    verification=dict(passed=True,protected_and_input_files=len(pins),containers=containers,
                      owned_diagnostic_jobs_exited=True,hesai=hesai,
                      original_CP56_world_origin_gate_passed=False,
                      graph_optimizer_calls=0,source_replays=0,production_changes=0,
                      disk_free_bytes=shutil.disk_usage(H).free,
                      time=datetime.datetime.now().astimezone().isoformat())
    save('verification.json',verification)
    retained={str(p):sha(p) for p in H.rglob('*') if p.is_file() and '__pycache__' not in p.parts
              and p.name not in ['complete.json','retained_manifest.json']}
    save('retained_manifest.json',retained)
    complete=dict(completed=True,suspect1_closed=False,full_zigzag_root_cause_proved=False,
                  next_suspect_started=False,production_fix=False,graph_optimizer_calls=0,
                  source_replays=0,verification=verification,
                  counts=findings['counts'],retained_files=len(retained),
                  report_sha256=sha(H/'REPORT.txt'),manifest_sha256=sha(H/'retained_manifest.json'))
    save('complete.json',complete)
    print(json.dumps(complete,indent=2))


if __name__=='__main__':main()
