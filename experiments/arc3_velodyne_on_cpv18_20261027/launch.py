"""Launch the explicitly authorized finite ON controller once, with owned inhibitors."""
from pathlib import Path
import datetime,fcntl,json,subprocess
H=Path(__file__).resolve().parent
def main():
 lock=(H/'launch.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
 assert not (H/'launch.json').exists() and not (H/'status.json').exists()
 assert json.loads((H/'prepared.json').read_text())['passed']
 assert json.loads((H/'reporter_preflight.json').read_text())['passed']
 initial=subprocess.check_output(['docker','inspect','-f','{{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'],text=True).split()
 assert initial==['exited','exited'],initial
 command=['systemd-inhibit','--what=sleep:idle:handle-lid-switch','--mode=block',
  '--who=ARC3-CPV18','--why=Authorized standalone ARC-3 CBS ON capture and cleanup',
  'gnome-session-inhibit','--app-id','ARC3-CPV18',
  '--reason','Authorized standalone ARC-3 CBS ON capture and cleanup','--inhibit','suspend:idle',
  'python3',str(H/'run_on.py')]
 with (H/'launch.json').open('x') as receipt,(H/'on.log').open('x') as log:
  job=subprocess.Popen(command,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
  result=dict(started=datetime.datetime.now().astimezone().isoformat(),launcher_pid=job.pid,command=command,
              original_containers=initial,standalone_ON=True,OFF_remains_failed=True)
  json.dump(result,receipt,indent=2)
 print(json.dumps(result,indent=2))
if __name__=='__main__':main()
