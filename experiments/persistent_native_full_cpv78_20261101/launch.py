"""Launch the single finite controller once with suspend inhibition."""
import datetime,json,os,subprocess
from pathlib import Path
H=Path(__file__).resolve().parent
assert (H/'prepared.json').exists() and not (H/'launch.json').exists() and not (H/'replay_attempt').exists()
args=['systemd-inhibit','--what=sleep','--mode=block','--who=CP-V78',
      '--why=Authorized full EIG2 native-matching persistent test','python3','-B',str(H/'controller.py')]
with (H/'controller.log').open('x') as log:
    p=subprocess.Popen(args,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,
                       start_new_session=True,env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1'))
receipt=dict(launcher_pid=os.getpid(),owned_supervisor_pid=p.pid,args=args,
             started=datetime.datetime.now().astimezone().isoformat(),single_finite_pair=True)
with (H/'launch.json').open('x') as f:json.dump(receipt,f,indent=2)
print(json.dumps(receipt,indent=2))
