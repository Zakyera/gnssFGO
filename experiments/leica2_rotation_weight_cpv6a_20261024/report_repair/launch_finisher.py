"""Attach one finite saved-data report continuation to the existing capture."""
from pathlib import Path
import datetime,json,subprocess
R=Path(__file__).resolve().parent
assert not (R/'launch.json').exists() and not (R/'start.json').exists()
assert json.loads((R.parent/'status.json').read_text())['state']=='running'
command=['systemd-inhibit','--what=sleep:idle:handle-lid-switch','--mode=block',
 '--who=LEICA2-CPV6-REPORT','--why=Finish authorized saved-data experiment report',
 'gnome-session-inhibit','--app-id','LEICA2-CPV6-REPORT','--reason','Finish authorized saved-data experiment report',
 '--inhibit','suspend:idle','python3',str(R/'finish_saved_report.py')]
with (R/'controller.log').open('x') as f:p=subprocess.Popen(command,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
with (R/'launch.json').open('x') as f:json.dump(dict(pid=p.pid,command=command,started=datetime.datetime.now().astimezone().isoformat(),persistent_automation=False),f,indent=2)
print(p.pid)
