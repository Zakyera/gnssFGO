"""Read-only verification plus new closeout record; does not stop or launch jobs."""
from datetime import datetime, timezone
import json
import subprocess
import tomllib
from driver import H,R,P,sha,save,verify

validation=json.loads((R/'validation.json').read_text())
assert validation['passed']
assert not (P/'full_off_control/complete.json').exists()
assert not (P/'full_on_source').exists()
containers={name:subprocess.check_output(['docker','inspect','--format','{{.State.Status}}',name],text=True).strip()
    for name in ('cbsms_ws_gpu','ubuntu-jammy-humble-nvidia')}
assert all(x=='exited' for x in containers.values()),containers
automation=tomllib.loads(open('/home/yeranis/.codex/automations/complete-frozen-eth1-live-wnoa-comparison/automation.toml').read())
assert automation['status']=='PAUSED'
subprocess.run(['git','-C',str(H.parent.parent),'diff','--check'],check=True)
result=dict(passed=True,closed_utc=datetime.now(timezone.utc).isoformat(),protected_unchanged=verify(),
    containers=containers,heartbeat_status=automation['status'],fresh_source_run=False,default_promoted=False,
    validation_sha256=sha(R/'validation.json'),candidate_sha256=sha(R/'live'),
    original_receiver_sha256=sha('/home/yeranis/results/gnssfgo_fixed_lag_failure_fl4e_20261014/candidate_live'),
    files={str(p.relative_to(H)):sha(p) for p in H.iterdir() if p.is_file()},
    gates={x:json.loads((R/(x+'.json')).read_text())['returncode'] for x in
      ('build_focused','test_focused','build_legacy','test_legacy','build_sanitize','test_sanitize','build_live','replay')})
assert all(rc==0 for rc in result['gates'].values())
save(H/'closeout.json',result)
print(json.dumps({k:v for k,v in result.items() if k!='files'},indent=2))
