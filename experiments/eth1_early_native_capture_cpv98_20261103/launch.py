"""One exclusive bounded diagnostic launch. Never retries."""
import datetime,json,os,subprocess
from pathlib import Path
H=Path(__file__).resolve().parent
assert json.loads((H/'prepared.json').read_text())['passed']
assert not (H/'start.json').exists() and not (H/'launch.json').exists()
command=['systemd-inhibit','--what=sleep','--mode=block','--who=ETH1-CPV98','--why=Authorized12s ETH1 passive native checkpoint diagnosis','python3','-B',str(H/'controller.py')]
with (H/'launch.json').open('x') as f:
    with (H/'controller.log').open('x') as log:
        p=subprocess.Popen(command,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True,env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1'))
    receipt=dict(launcher_pid=os.getpid(),supervisor_pid=p.pid,proc_start=Path(f'/proc/{p.pid}/stat').read_text().split()[21],command=command,started=datetime.datetime.now().astimezone().isoformat());json.dump(receipt,f,indent=2)
print(json.dumps(receipt))
