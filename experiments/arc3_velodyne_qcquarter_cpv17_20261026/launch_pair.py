"""Launch exactly one authorized finite ARC-3 OFF/ON pair with sleep protection."""
from pathlib import Path
import datetime,fcntl,json,subprocess
H=Path(__file__).resolve().parent
lock=(H/'launch.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
assert json.loads((H/'prelaunch_review.json').read_text())['passed']
assert not (H/'launch.json').exists() and not (H/'pair.log').exists()
assert not (H/'arc3/status.json').exists() and not (H/'arc3/start.json').exists()
cmd=['systemd-inhibit','--what=sleep:idle:handle-lid-switch','--mode=block',
     '--who=ARC3-CPV17','--why=Authorized finite ARC3 Velodyne Qc0.25 OFF ON experiment',
     'gnome-session-inhibit','--app-id','ARC3-CPV17','--reason','Finite ARC3 OFF ON experiment',
     '--inhibit','suspend:idle','python3',str(H/'arc3/run_pair.py')]
with (H/'pair.log').open('x') as log:
    p=subprocess.Popen(cmd,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
data=dict(pid=p.pid,command=cmd,started=datetime.datetime.now().astimezone().isoformat(),
          scope='One OFF, gates, ON only on pass, gates, saved scoring, cleanup; no retry')
with (H/'launch.json').open('x') as f:json.dump(data,f,indent=2)
print(json.dumps(data,indent=2))
