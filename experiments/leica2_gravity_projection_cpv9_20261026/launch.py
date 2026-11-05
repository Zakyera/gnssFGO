"""One finite candidate controller with process-scoped sleep inhibitors."""
from pathlib import Path
import datetime,json,os,subprocess
H=Path(__file__).resolve().parent
assert not (H/'launch.json').exists() and not (H/'status.json').exists()
assert json.loads((H/'prepared.json').read_text())['passed']
assert subprocess.check_output(['docker','inspect','-f','{{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'],text=True).split()==['exited','exited']
command=['systemd-inhibit','--what=sleep:idle:handle-lid-switch','--mode=block',
 '--who=LEICA2-CPV9','--why=Authorized finite Leica2 gravity-projection experiment',
 'gnome-session-inhibit','--app-id','LEICA2-CPV9','--reason',
 'Authorized finite Leica2 gravity-projection experiment','--inhibit','suspend:idle',
 'python3',str(H/'run_candidate.py')]
with (H/'controller.log').open('x') as log:
 process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
receipt=dict(pid=process.pid,command=command,started=datetime.datetime.now().astimezone().isoformat(),persistent_automation=False)
with (H/'launch.json').open('x') as f:json.dump(receipt,f,indent=2)
print(json.dumps(receipt))
