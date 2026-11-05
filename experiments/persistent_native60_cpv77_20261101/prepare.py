"""Create fresh private replay scaffolding; never execute old controllers."""
import ast,hashlib,json,shutil,xml.etree.ElementTree as ET
from pathlib import Path
H=Path(__file__).resolve().parent; E=H.parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3');K=W/'runs'/H.name;R=K/'replay'
K73=W/'runs/retention_readiness_cpv73_20261031';K76=W/'runs/persistent_compatibility_cpv76_20261101'
S=W/'runs/eig2_hesai_refresh_full_cpv49_20261030/prepared/full_on_source'
assert not K.exists();R.mkdir(parents=True);(K/'private_build').mkdir()
def save(p,v):
    with p.open('x') as f:json.dump(v,f,indent=2)
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def replace(s,a,b):
    assert a in s,a
    return s.replace(a,b)
v=json.loads((E/K76.name/'verification_fixture1.json').read_text());assert v['passed']
for name,digest in v['libraries'].items():
    assert sha(K76/'private_build'/name)==digest
    shutil.copy2(K76/'private_build'/name,K/'private_build'/name)
save(K/'binary_gate.json',dict(passed=True,source=str(E/K76.name/'verification_fixture1.json'),sha256=v['libraries']))
shutil.copytree(S/'kimera_profile',R/'kimera_profile')
for p in (K73/'replay').glob('*.launch'):
    s=p.read_text().replace('--cbs_experimental_native_imu_mean_alignment=false','--cbs_experimental_native_imu_mean_alignment=true')
    s='\n'.join(l for l in s.splitlines() if 'CPV70_RECORDED_UPDATE_SCHEDULE' not in l and 'CPV73_REPLAY_READY_TIMEOUT_MS' not in l)+'\n'
    if p.name=='kimera_vio_ros.launch':
        s=replace(s,'<env name="CPV69_RETAIN_COMPAT" value="$(env CPV69_RETAIN_COMPAT)"/>','<env name="CPV69_RETAIN_COMPAT" value="$(env CPV69_RETAIN_COMPAT)"/>\n    <env name="CPV76_PERSISTENT_COMPAT" value="$(env CPV76_PERSISTENT_COMPAT)"/>')
    (R/p.name).write_text(s);ET.parse(R/p.name)
s=(K73/'replay/run_case.py').read_text().replace("['remove','retain']","['temporary','persistent']")
s=replace(s,"if mode=='retain': assert json.loads((R/'remove_capture/validation.json').read_text())['valid']","if mode=='persistent': assert json.loads((R/'temporary_capture/validation.json').read_text())['valid']")
s=s.replace('11573','11577').replace("FL4M_RETIREMENT=mode","FL4M_RETIREMENT='remove'")
s=replace(s,"CPV69_RETAIN_COMPAT='1' if mode=='retain' else '0'","CPV69_RETAIN_COMPAT='0',CPV76_PERSISTENT_COMPAT='1' if mode=='persistent' else '0'")
s=replace(s,"os.environ['CPV70_RECORDED_UPDATE_SCHEDULE']=str(R/'recorded_updates.txt')","os.environ.pop('CPV70_RECORDED_UPDATE_SCHEDULE',None)")
s=replace(s,"os.environ['CPV73_REPLAY_READY_TIMEOUT_MS']='2000'","os.environ.pop('CPV73_REPLAY_READY_TIMEOUT_MS',None)")
s=replace(s,"'cbs_odom_factor_mode:=active_window_temporary'","'cbs_odom_factor_mode:='+('persistent' if mode=='persistent' else 'active_window_temporary')")
s=replace(s,"('false' if k=='cbs_experimental_native_imu_mean_alignment' else v)","('persistent' if mode=='persistent' and k=='cbs_odom_factor_mode' else v)")
s=replace(s,"'CPV69_RETAIN_COMPAT','CPK9_PRIOR_REFRESH'","'CPV69_RETAIN_COMPAT','CPV76_PERSISTENT_COMPAT','CPK9_PRIOR_REFRESH'")
s=replace(s,"env['FL4M_RETIREMENT']==mode","env['FL4M_RETIREMENT']=='remove'")
s=replace(s,"assert env['CPV69_RETAIN_COMPAT']==('1' if mode=='retain' else '0')","assert env['CPV69_RETAIN_COMPAT']=='0'\n    assert env['CPV76_PERSISTENT_COMPAT']==('1' if mode=='persistent' else '0')")
s=replace(s,"assert env['CPV70_RECORDED_UPDATE_SCHEDULE']==str(R/'recorded_updates.txt')","assert 'CPV70_RECORDED_UPDATE_SCHEDULE' not in env")
s=replace(s,"assert env['CPV73_REPLAY_READY_TIMEOUT_MS']=='2000'","assert 'CPV73_REPLAY_READY_TIMEOUT_MS' not in env")
s=replace(s,"assert master.getSystemState()[1] and not any('glim' in n for n in run(['rosnode','list']).splitlines())","system_state=master.getSystemState()\n    assert system_state[1] and not any('glim' in n for n in run(['rosnode','list']).splitlines())\n    assert not any(nodes for topic,nodes in system_state[1] if topic in ('/kimera/cbs/odom_belief_out','/glim/cbs/odom_belief_in')), 'unexpected outgoing consumer'\n    (D/'ros_graph.json').write_text(json.dumps(system_state,indent=2))")
s=s.replace('355.02','60.0').replace("str(R/'aligned_inputs.bag')","str(R/'raw_inputs.bag')").replace('+2100','+480')
s=replace(s,"'active_window_rollback','CPV40_FATAL','process has died'","'active_window_rollback','CPV40_FATAL','process has died','CPV76 incompatible','CPV76 invalid','CPV76 marginal','CPV76 missing','CPV76 wrong','CPV76 changed','CPV76 replaced','CPV76 native','CPV76 dangling','CPV76 persistent residual'")
ast.parse(s);(R/'run_case.py').write_text(s)
for name in ('prepare_inputs.py','validate.py'):shutil.copyfile(H/name,R/name)
# Adapt saved-only scoring, retaining the exact established EVO implementation.
s=(E/'retention_update_gate_cpv70_20261031/evaluate.py').read_text()
a=s.index('    absorptions=[');b=s.index('    timing=[]',a)
s=s[:a]+s[b:]
s=s.replace("absorption_boundaries=len(absorptions)","absorption_boundaries=None")
s=s.replace("'remove'","'temporary'").replace("'retain'","'persistent'").replace("task='CP-V70'","task='CP-V77'")
s=s.replace("conditional Kimera-only60s remove/retain; saved CP66 ON history held fixed; recorded-update delivery","conditional Kimera-only60s temporary/general persistent; complete CP49 raw wire stream; native matching and mean alignment")
ast.parse(s);(H/'evaluate.py').write_text(s)
print('Fresh scaffold created; no estimator started.')
