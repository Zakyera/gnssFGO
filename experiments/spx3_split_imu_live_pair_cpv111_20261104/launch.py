"""Start only the authorized finite OFF/ON pair, exactly once."""
import datetime,json,os,subprocess
from pathlib import Path
H=Path(__file__).resolve().parent
assert json.loads((H/'prepared.json').read_text())['passed']
assert not (H/'launch.json').exists() and not (H/'start.json').exists()
cmd=['systemd-inhibit','--what=sleep','--mode=block','--who=SPX3-CPV111',
     '--why=User-authorized SPX-3 split-IMU full OFF/ON with fusion',
     'python3','-B',str(H/'controller.py')]
with (H/'launch.json').open('x') as f:
    with (H/'controller.log').open('x') as log:
        p=subprocess.Popen(cmd,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,
                           start_new_session=True,env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1'))
    receipt=dict(launcher_pid=os.getpid(),owned_supervisor_pid=p.pid,args=cmd,
        supervisor_proc_start=Path(f'/proc/{p.pid}/stat').read_text().rsplit(')',1)[1].split()[19],
        started=datetime.datetime.now().astimezone().isoformat(),fresh_pair=True,source_captures_authorized=2)
    json.dump(receipt,f,indent=2)
print(json.dumps(receipt,indent=2))
