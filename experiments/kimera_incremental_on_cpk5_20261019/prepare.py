"""Copy immutable CP-K4b assets and mechanically adapt only the isolated harness."""
import ast,hashlib,json,shutil,xml.etree.ElementTree as ET
from pathlib import Path
H=Path(__file__).resolve().parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
R=W/'runs/kimera_incremental_on_cpk5_20261019'
O=W/'runs/kimera_incremental_control_cpk4b_20261019'
S=W/'runs/gnssfgo_fixed_lag_live120_on_fl4v_20261017/prepared/live120_on_source'
def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        for b in iter(lambda:f.read(1048576),b''):h.update(b)
    return h.hexdigest()
assert not R.exists(),'Refuse duplicate preparation'
assert json.loads((O/'off_capture/validation.json').read_text())['passed']
protected=json.loads((O/'protected_before.json').read_text())
assert all(sha(p)==h for p,h in protected.items())
assert shutil.disk_usage(W).free>8*1024**3
R.mkdir()
shutil.copytree(O/'kimera_profile',R/'kimera_profile')
for n in ('grandtour_eth1_kimera_stereo.launch','kimera_vio_ros.launch',
          'kimera_vio_ros_newer_college_2021_stereo.launch','preflight_tests.json'):
    shutil.copy2(O/n,R/n)
    if n.endswith('.launch'):ET.parse(R/n)
for p in (O/'kimera_profile').rglob('*'):
    if p.is_file():assert sha(p)==sha(S/'kimera_profile'/p.relative_to(O/'kimera_profile'))
shutil.copy2(H/'PROTOCOL.txt',R/'PROTOCOL.txt')
shutil.copy2(H/'extract_inputs.py',R/'extract_inputs.py')
c=(O/'run_case.py').read_text()
def edit(a,b):
    global c
    assert a in c,a
    c=c.replace(a,b)
edit('bounded native-incremental OFF control','bounded ordinary-incremental saved-input ON diagnostic')
edit("D=R/'off_capture'","D=R/'on_capture'")
edit('11456','11457');edit('cpk4_preflight','cpk5_preflight');edit('CP-K4','CP-K5')
edit("args=[s.replace(str(K),str(R)) for s in ref['launch']]",
     "args=[s.replace(str(K),str(R)).replace('/off_capture/','/on_capture/').replace('cbs_belief_receive_enable:=false','cbs_belief_receive_enable:=true') for s in ref['launch']]")
edit("params['cbs_belief_receive_enable'] is False","params['cbs_belief_receive_enable'] is True")
edit("str(D/'outputs.bag'),'/kimera_vio_ros/odometry']","str(D/'outputs.bag'),'/kimera_vio_ros/odometry','/kimera/cbs/odom_belief_in']")
edit("play=start(old_play,'play')", "old_play.insert(old_play.index('--topics'),str(R/'raw_glim_inputs.bag'))\n    old_play.append('/kimera/cbs/odom_belief_in')\n    play=start(old_play,'play')")
edit('OFF CONTROL: FL4Q=0','ON SAVED-INPUT DIAGNOSTIC: FL4Q=0')
ast.parse(c)
(R/'run_case.py').write_text(c)
for p in O.rglob('*'):
    if p.is_file():protected[str(p)]=sha(p)
for n in ('recordings/cbs.bag','estimators.log','effective_mode_check.json','resolved_glim_params.yaml','resolved_kimera_params.yaml'):
    protected[str(S/n)]=sha(S/n)
(R/'protected_before.json').write_text(json.dumps(protected,indent=2))
(R/'preparation.json').write_text(json.dumps(dict(protected_count=len(protected),
    same_profile_as_off_and_saved_on=True,ordinary_backend_both=True,source_edit=False,
    script_changes_only=True,source_capture_limit=1),indent=2))
print('PREPARED',R,len(protected))
