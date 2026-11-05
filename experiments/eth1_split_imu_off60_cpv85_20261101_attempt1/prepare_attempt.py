"""Fresh-root mechanical launch repair; preserve every failed-attempt artifact."""
from pathlib import Path
import shutil
H=Path(__file__).resolve().parent
old_name='eth1_split_imu_off60_cpv85_20261101'
OLD=H.parent/old_name
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
Q=W/'runs'/old_name/'prepared';P=W/'runs'/H.name/'prepared'
assert not P.exists() and (OLD/'closeout.json').exists() and (OLD/'failed.json').exists()
P.mkdir(parents=True)
for name in ['source_runner_on.sh','source.env','verify_effective_on.py','verify_hesai_startup.py',
             'map_guard.py','coverage_checks.py','coverage_split.py','active_window_on.launch',
             'glim_profile_on.launch','grandtour_eth1_kimera_stereo.launch','kimera_vio_ros.launch',
             'kimera_vio_ros_newer_college_2021_stereo.launch']:
    shutil.copy2(Q/name,P/name)
    t=(P/name).read_text().replace(old_name,H.name).replace('11586','11587')
    if name=='source_runner_on.sh':
        assert t.count('glim_cbs_mode=observe_only')==1
        t=t.replace('glim_cbs_mode=observe_only','glim_cbs_mode=inject_persistent')
    if name=='verify_effective_on.py':
        assert t.count("glim_cbs_mode='observe_only'")==1
        t=t.replace("glim_cbs_mode='observe_only'","glim_cbs_mode='inject_persistent'")
    (P/name).write_text(t)
for name in ('private_build','input_profiles'):shutil.copytree(Q/name,P/name)
for name in ['author_alphasense_imu.yaml','audit_inputs.py','evaluate.py','controller.py','launch.py','freeze.py','PROTOCOL.txt']:
    shutil.copy2(OLD/name,H/name)
    t=(H/name).read_text().replace('11586','11587')
    if name=='freeze.py':
        needle="for p,d in pins.items():assert sha(p)==d,p"
        assert needle in t
        t=t.replace(needle,"""# Pin the failed pre-playback attempt as well as all earlier protected evidence.
prior=E/'eth1_split_imu_off60_cpv85_20261101'
assert read(prior/'closeout.json')['passed'] and (prior/'failed.json').exists()
for root in (prior,W/'runs'/prior.name/'prepared'):
    for p in root.rglob('*'):
        if p.is_file() and '__pycache__' not in p.parts:pins[str(p)]=sha(p)
"""+needle)
    (H/name).write_text(t)
print(P)
