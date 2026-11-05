"""Exclusive detached launch of the finite, user-authorized overnight campaign."""
from pathlib import Path
import ast,datetime,json,os,subprocess,sys
H=Path(__file__).resolve().parent
repair=sys.argv[1:]==['--verified-cli-repair']
assert not sys.argv[1:] or repair
assert not (H/'start.json').exists() and not (H/'status.json').exists()
receipt='launch.json';log_name='campaign.log'
if repair:
 old=json.loads((H/'launch.json').read_text())
 assert not Path('/proc/'+str(old['pid'])).exists(),'Earlier launcher still exists'
 assert (H/'campaign.log').read_text()=='Failed to execute --app-id=VELODYNE-CPV4-CAMPAIGN\ngnome-session-inhibit failed with exit status 1.\n'
 assert subprocess.check_output(['docker','inspect','-f','{{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'],text=True).split()==['exited','exited']
 receipt='launch_cli_repair.json';log_name='campaign_cli_repair.log'
assert not (H/receipt).exists() and not (H/log_name).exists()
ast.parse((H/'run_campaign.py').read_text())
config=json.loads((H/'campaign.json').read_text())
for c in config['sequences']:
 d=H/c['slug'];assert (d/'prepared.json').exists() or (d/'blocked.json').exists(),d
cmd=['systemd-inhibit','--what=sleep:idle:handle-lid-switch','--mode=block',
 '--who=VELODYNE-CPV4-CAMPAIGN','--why=Authorized finite overnight Velodyne campaign',
 'gnome-session-inhibit','--app-id','VELODYNE-CPV4-CAMPAIGN',
 '--reason','Authorized finite overnight Velodyne campaign','--inhibit','suspend:idle',
 'python3',str(H/'run_campaign.py')]
with (H/log_name).open('x') as log:
 p=subprocess.Popen(cmd,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
data=dict(pid=p.pid,command=cmd,started=datetime.datetime.now().astimezone().isoformat(),log=log_name,verified_cli_repair=repair)
with (H/receipt).open('x') as f:json.dump(data,f,indent=2)
print(json.dumps(data,indent=2))
