"""One source-only capture, in existing container; no retries or fusion."""
import json,os,socket,subprocess,signal,time,hashlib
from pathlib import Path
root=Path('/workspace/cbs_gtsam4.3/runs/fixed_lag_same_graph_fl4l_20261015')
control=root/'control_capture'; control.mkdir()
assert not (root/'on_source_capture').exists() and not (root/'capture').exists()
with socket.socket() as sock: sock.bind(('127.0.0.1',11435))
def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()
identities={str(p):sha(p) for p in (root/'private_build').iterdir() if p.is_file()}
(control/'runtime_hashes.json').write_text(json.dumps(identities,indent=2))
command='set -e; export CBSMS_RUN_NAME=on_source_capture CBSMS_CBS_ENABLE=true; source "$1/source.env"; exec bash "$1/source_runner_on.sh"'
start=time.time()
with (control/'source.log').open('x') as log:
    p=subprocess.Popen(['bash','-c',command,'fl4l',str(root)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    (control/'started.json').write_text(json.dumps({'pid':p.pid,'start_unix':start,'duration_s':94,'fusion':False},indent=2))
    print('source-only capture started pid='+str(p.pid),flush=True)
    try:
        code=p.wait(timeout=1500)
    except BaseException:
        os.killpg(p.pid,signal.SIGINT)
        try: p.wait(timeout=40)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid,signal.SIGTERM); p.wait(timeout=15)
        (control/'failed.txt').write_text('controller interruption/timeout; no retry\n')
        raise
status=root/'capture/update_438/status.txt'
ok=code==0 and (root/'on_source_capture/complete.marker').exists() and status.exists() and 'complete' in status.read_text().splitlines()
result={'exit':code,'capture_complete':ok,'wall_sec':time.time()-start}
(control/('complete.json' if ok else 'failed.json')).write_text(json.dumps(result,indent=2))
print(json.dumps(result),flush=True)
if not ok: raise SystemExit(1)
