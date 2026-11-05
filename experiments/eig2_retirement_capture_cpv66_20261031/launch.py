"""One finite matched pair with scoped system/session sleep inhibition."""
import json,os,subprocess,time
from pathlib import Path
H=Path(__file__).resolve().parent
assert (H/'prepared.json').exists() and not (H/'start.json').exists()
with (H/'launch.json').open('x') as f:
    cmd=['systemd-inhibit','--what=sleep:idle:handle-lid-switch','--mode=block','--who=EIG2-CPV66',
        '--why=Authorized matched60s retirement observation','gnome-session-inhibit','--app-id','EIG2-CPV66',
        '--reason','Authorized matched60s retirement observation','--inhibit','suspend:idle','python3',str(H/'controller.py')]
    with (H/'controller.log').open('xb') as log:
        job=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,stdin=subprocess.DEVNULL,start_new_session=True)
    receipt=dict(pid=job.pid,pgid=os.getpgid(job.pid),command=cmd,wall_ns=time.time_ns())
    json.dump(receipt,f,indent=2)
print(json.dumps(receipt),flush=True)
