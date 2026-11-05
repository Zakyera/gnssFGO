"""Fresh CP84 startup repair: preserve failed attempt, scripts and executable modes."""
from pathlib import Path
import json,shutil
H=Path(__file__).resolve().parent;E=H.parent
OLD=E/'eig2_symmetric_persistent_full_cpv84_20261101'
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3');O=W/'runs'/OLD.name/'prepared';P=W/'runs'/H.name/'prepared'
BASE=W/'runs/eig2_persistent_live_cpv80_20261101/prepared'
def read(p):return json.loads(p.read_text())
assert read(OLD/'failed.json')['stage']=='capture_on' and read(OLD/'closeout.json')['passed']
assert not (O/'full_on_source').exists() and not (O/'full_on_receiver').exists()
assert read(O/'full_on_control/owned_job_cleanup.json')=={'master':0}
assert not P.exists();P.mkdir(parents=True)
def rebase(s):return s.replace(OLD.name,H.name).replace('11584','11585').replace('EIG2-CPV84','EIG2-CPV84-A1')
def put(p,s):
    with p.open('x') as f:f.write(s)
files='active_window_on.launch coverage_checks.py eof_boundary.py experiment.py expiry_audit.py full_on.json glim_profile_on.launch grandtour_eth1_kimera_stereo.launch kimera_vio_ros.launch kimera_vio_ros_newer_college_2021_stereo.launch map_guard.py preflight_on.py receiver_cpk19.sh run_case_on.py source.env source_runner_on.sh validate_candidate.py validate_case.py validate_hesai.py validate_local.py verify_effective_on.py verify_hesai_startup.py input_audit.json'.split()
for name in files:
    s=rebase((O/name).read_text())
    if name=='full_on.json':s=s.replace('eig2_cpv84_hesai_on_20261101','eig2_cpv84_attempt1_hesai_on_20261101')
    put(P/name,s);(P/name).chmod((BASE/name).stat().st_mode & 0o777)
for name in ('input_profiles','private_build','transport_tools'):
    shutil.copytree(O/name,P/name,ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
protocol=P/'transport_tools/live_receiver_protocol.py';protocol.write_text(rebase(protocol.read_text()))
for name in ('controller.py','launch.py','evaluate.py','verify_report.py','prepare_host.py','post_closeout.py','PROTOCOL.txt'):
    s=rebase((OLD/name).read_text())
    if name=='prepare_host.py':
        s=s.replace('assert not (H/\'prepared.json\').exists() and not V.exists()', "assert not (H/'prepared.json').exists() and not V.exists()\nassert (P/'receiver_cpk19.sh').stat().st_mode & 0o111, 'wrapper executable permission missing'")
        needle="for p,d in pins.items():assert sha(p)==d,p"
        addition="""FAILED=E/'eig2_symmetric_persistent_full_cpv84_20261101'
FO=W/'runs'/FAILED.name/'prepared'
assert read(FAILED/'closeout.json')['passed']
assert not (FO/'full_on_source').exists() and not (FO/'full_on_receiver').exists()
pins.update(read(FO/'protected_manifest.json'))
for item in ('failed.json','closeout.json','start.json','launch.json','status.json','capture_on.log','controller.log','prepare_host.log','prepare_host_retry1.log'):
    pins[str(FAILED/item)]=sha(FAILED/item)
for item in (FO/'full_on_control').iterdir():
    if item.is_file():pins[str(item)]=sha(item)
"""
        assert needle in s;s=s.replace(needle,addition+needle)
    if name=='PROTOCOL.txt':s+='\nFRESH STARTUP REPAIR ATTEMPT1: initial CP84 stopped before receiver or sources\nstarted because wrapper file was664 instead of755. Original failed attempt and\nall hashes/logs preserved. Only executable permissions and fresh paths/port/session\nchanged. No sensor replay or numerical retry. Require executable wrapper in\npreflight; both containers restored before this fresh attempt.\n'
    put(H/name,s)
print(json.dumps(dict(prepared=str(P),original_failure_preserved=True,source_captures_before_attempt=0,wrapper_mode=oct((P/'receiver_cpk19.sh').stat().st_mode & 0o777)),indent=2))
