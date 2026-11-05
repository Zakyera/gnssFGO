"""Author a fresh CP83 60s harness from immutable CP80, after CP82 passes.

This copies/retargets only named harness files. It never launches an estimator.
"""
import hashlib,json,shutil
from pathlib import Path
H=Path(__file__).resolve().parent;E=H.parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
OLD='eig2_persistent_live_cpv80_20261101'
NAME='eig2_symmetric_persistent60_cpv83_20261101'
S=W/'runs'/OLD/'prepared';D=W/'runs'/NAME/'prepared';N=E/NAME
C=W/'runs'/H.name/'release_4'
assert json.loads((H/'complete.json').read_text())['passed']
assert not N.exists() and not D.parent.exists()
N.mkdir();D.mkdir(parents=True)
def write(p,text):
    with p.open('x') as f:f.write(text)
def edit(text,a,b,count=None):
    assert a in text,a
    if count is not None:assert text.count(a)==count,(a,text.count(a),count)
    return text.replace(a,b)
def rebase(text):
    return text.replace(OLD,NAME).replace('CP-V80','CP-V83').replace('CPV80','CPV83').replace('cpv80','cpv83').replace('11580','11583').replace('355.02','60.0').replace('353.02','58.0')
names=['active_window_on.launch','coverage_checks.py','eof_boundary.py','experiment.py',
       'expiry_audit.py','full_on.json','glim_profile_on.launch','grandtour_eth1_kimera_stereo.launch',
       'kimera_vio_ros.launch','kimera_vio_ros_newer_college_2021_stereo.launch','map_guard.py',
       'preflight_on.py','receiver_cpk19.sh','run_case_on.py','source.env','source_runner_on.sh',
       'validate_candidate.py','validate_case.py','validate_hesai.py','validate_local.py',
       'verify_effective_on.py','verify_hesai_startup.py']
candidate='/workspace/cbs_gtsam4.3/runs/'+H.name+'/release_4'
for name in names:
    text=rebase((S/name).read_text())
    if name=='coverage_checks.py':
        text=edit(text,'3547','597');text=edit(text,'3550','600')
    if name in ('source_runner_on.sh','validate_case.py','validate_hesai.py'):
        text=edit(text,'3517','590')
    if name=='validate_local.py':text=edit(text,'first>=3530','first>=590')
    if name=='validate_candidate.py':
        text=edit(text,'len(times)>1700','len(times)>280')
        text=edit(text,'354<=(times[-1]-epoch)*1e-9<=355.12','59<=(times[-1]-epoch)*1e-9<=60.1')
        marker='    assert effective["kimera"]["cbs_odom_factor_mode"] == "persistent"'
        text=edit(text,marker,marker+'\n    assert effective["glim"]["glim_cbs_mode"] == "inject_persistent"\n    assert effective["glim_persistent_compat"] == "1"\n    assert "injected_persistent" in log\n    assert "active_window_temporary_graph_queued" not in log\n    assert "GLIM_CBS_ACTIVE_WINDOW_FACTOR_ROW,removed" not in log')
    if name in ('verify_effective_on.py','validate_case.py','source_runner_on.sh'):
        text=edit(text,'active_window_temporary','inject_persistent') if name!='source_runner_on.sh' else text.replace('glim_cbs_mode=active_window_temporary','glim_cbs_mode=inject_persistent').replace('active_window_temporary_graph_queued','injected_persistent')
    if name=='map_guard.py':
        text=edit(text,'/workspace/cbs_gtsam4.3/runs/local_revision_covariance_cpi1_20261019',candidate)
    if name=='glim_profile_on.launch':
        marker='<env name="GLIM_SHARED_NATIVE_PROVIDER" value="shared_native"/>'
        text=edit(text,marker,marker+'\n        <env name="CPV82_GLIM_PERSISTENT_COMPAT" value="1"/>')
        marker='/workspace/cbs_gtsam4.3/runs/local_revision_covariance_cpi1_20261019:'
        text=edit(text,marker,candidate+':'+marker)
    if name=='verify_effective_on.py':
        marker="assert env(glim_pid)['GLIM_SHARED_NATIVE_PROVIDER']=='shared_native'"
        text=edit(text,marker,marker+"\nassert env(glim_pid)['CPV82_GLIM_PERSISTENT_COMPAT']=='1'\nassert 'CPV82_GLIM_PERSISTENT_COMPAT' not in env(pid)")
        marker="import hashlib"
        text=edit(text,marker,marker+"\nassert hashlib.sha256(Path('"+candidate+"/libglim_ros.so').read_bytes()).hexdigest()=='aecc729f4bf51548950b8cb997c3dcd474e30cabe23785c4011e6b22f7dbe918'")
        text=edit(text,"kimera_maps=loaded,glim_maps=glim_loaded,","kimera_maps=loaded,glim_maps=glim_loaded,glim_persistent_compat=env(glim_pid)['CPV82_GLIM_PERSISTENT_COMPAT'],")
        text=text.replace('GLIM temporary / Kimera general persistent','GLIM persistent / Kimera general persistent')
    text=text.replace('full-usable60.0-second','diagnostic-prefix60.0-second')
    write(D/name,text);shutil.copymode(S/name,D/name)
for folder in ('input_profiles','private_build','transport_tools'):
    shutil.copytree(S/folder,D/folder,ignore=shutil.ignore_patterns('__pycache__'))
for p in (D/'transport_tools').rglob('*.py'):
    p.write_text(rebase(p.read_text()))
audit_source=W/'runs/eig2_retirement_capture_cpv66_20261031/prepared/input_audit.json'
audit=json.loads(audit_source.read_text());audit['reused_verified_60s_audit']=str(audit_source)
write(D/'input_audit.json',json.dumps(audit,indent=2))
for name in ('controller.py','evaluate.py','verify_report.py','launch.py','prepare_host.py'):
    text=rebase((E/OLD/name).read_text())
    if name=='controller.py':
        text=text.replace('One full live ON','One 60s live ON').replace('Full live ON','60s live ON')
        text=text.replace('full_live_on=True','completed_requested_live_window=True')
    if name in ('evaluate.py','verify_report.py'):
        text=edit(text,'active_window_temporary','inject_persistent')
    if name=='evaluate.py':
        text=text.replace('FULL LIVE EIG-2','60s LIVE EIG-2').replace('ONE full60.0s','ONE requested60.0s').replace('GLIM remains inject_persistent','GLIM CP82 inject_persistent')
    if name=='verify_report.py':
        text=edit(text,"v['reference_path_length_m']>150","v['reference_path_length_m']>25")
        text=edit(text,"['pairs']>100","['pairs']>40")
    if name=='launch.py':text=text.replace('full live EIG2 persistent Kimera GLIM fusion','60s live EIG2 persistent BOTH GLIM Kimera fusion')
    if name=='prepare_host.py':
        text=edit(text,'required=int(source_bytes*1.25)+8*1024**3+512*1024**2','required=int(source_bytes*(60.0/355.02)*1.5)+8*1024**3+512*1024**2')
        text=edit(text,'forecast_margin=1.25','forecast_margin=1.5, duration_ratio=60.0/355.02')
        marker="pins=read(OLD/'protected_before.json')"
        extra="\nCP82=E/'"+H.name+"'\nassert read(CP82/'complete.json')['passed']\npins.update(read(CP82/'complete.json')['output_sha256'])\npins[str(CP82/'complete.json')]=sha(CP82/'complete.json')\nCP80=E/'"+OLD+"'\npins.update(read(CP80/'complete.json')['output_sha256'])\npins[str(CP80/'complete.json')]=sha(CP80/'complete.json')"
        text=edit(text,marker,marker+extra)
        text=edit(text,'glim_cbs_mode=active_window_temporary','glim_cbs_mode=inject_persistent')
        text=edit(text,'Single full live bidirectional ON with GLIM temporary / Kimera persistent / concurrent fusion','Single 60s live bidirectional ON with BOTH persistent / concurrent fusion')
    write(N/name,text)
print(json.dumps(dict(authored=str(N),prepared=str(D),launched=False),indent=2))
