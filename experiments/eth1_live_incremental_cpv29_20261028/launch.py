"""Exclusive durable launch under a scoped sleep inhibitor; never retries."""
import json, os, subprocess, time
from pathlib import Path
H=Path(__file__).resolve().parent
assert not (H/'start.json').exists() and not (H/'status.json').exists()
receipt=H/'launch.json'
with receipt.open('x') as f:
    cmd=['systemd-inhibit','--what=sleep:idle:handle-lid-switch','--who=ETH1-CPV29',
         '--why=Authorized single full ETH1 Hesai incremental-output experiment',
         '--mode=block','--','python3',str(H/'run.py')]
    with (H/'controller.log').open('xb') as log:
        proc=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,stdin=subprocess.DEVNULL,start_new_session=True)
    data=dict(pid=proc.pid,pgid=os.getpgid(proc.pid),time=time.time(),command=cmd)
    json.dump(data,f,indent=2);f.write('\n')
print(json.dumps(data),flush=True)
