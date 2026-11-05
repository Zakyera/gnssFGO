"""Copy only static tested harnesses to fresh CP84 roots; never launch anything."""
from pathlib import Path
import difflib,json,shutil
H=Path(__file__).resolve().parent;E=H.parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3');P=W/'runs'/H.name/'prepared'
H80=E/'eig2_persistent_live_cpv80_20261101';P80=W/'runs'/H80.name/'prepared'
H83=E/'eig2_symmetric_persistent60_cpv83_20261101';P83=W/'runs'/H83.name/'prepared'
assert not P.exists() and not (H/'prepared.json').exists()
P.mkdir(parents=True)
def rebase(s):
    for old,port in ((H80.name,'11580'),(H83.name,'11583')):
        s=s.replace(old,H.name).replace(port,'11584')
    return s
def put(p,s):
    with p.open('x') as f:f.write(s)
files='active_window_on.launch coverage_checks.py eof_boundary.py experiment.py expiry_audit.py full_on.json glim_profile_on.launch grandtour_eth1_kimera_stereo.launch kimera_vio_ros.launch kimera_vio_ros_newer_college_2021_stereo.launch map_guard.py preflight_on.py receiver_cpk19.sh run_case_on.py source.env source_runner_on.sh validate_candidate.py validate_case.py validate_hesai.py validate_local.py verify_effective_on.py verify_hesai_startup.py input_audit.json'.split()
diff=[]
for name in files:
    original=(P80/name).read_text();s=rebase(original)
    if name in ('glim_profile_on.launch','map_guard.py','verify_effective_on.py'):
        s=rebase((P83/name).read_text())
        if name=='verify_effective_on.py':s=s.replace("== '60.0'","== '355.02'").replace('requested_s=60.0','requested_s=355.02')
    if name=='source_runner_on.sh':
        assert s.count('glim_cbs_mode=active_window_temporary')==3
        s=s.replace('glim_cbs_mode=active_window_temporary','glim_cbs_mode=inject_persistent')
        s=s.replace('GLIM_CBS_ODOM_INJECT_ROW.*active_window_temporary_graph_queued','GLIM_CBS_ODOM_INJECT_ROW.*injected_persistent')
        s=s.replace('cbs_factor_mode=glim_temporary_kimera_persistent','cbs_factor_mode=both_persistent')
        s=s.replace('# The pinned S4d covariance provider requires this bookkeeping mode even\n# when CBS-OFF disables the incoming subscription. No external factors exist.', '# CP82 explicitly restores existing persistent reception with shared_native;\n# the private opt-in is confined to the GLIM process.')
    if name=='validate_case.py':s=s.replace('active_window_temporary','inject_persistent')
    if name=='full_on.json':
        s=s.replace('CP-V80','CP-V84').replace('eig2_cpv80_','eig2_cpv84_').replace('Kimera persistent / GLIM temporary','BOTH GLIM and Kimera persistent')
    if name=='validate_candidate.py':
        s=(H83/'saved_validation_repair_1/validate_candidate.py').read_text()
        s=s.replace("R = Path('"+str(P83)+"')",'R = Path(__file__).resolve().parent')
        s=s.replace("Path('"+str(H83/'saved_validation_repair_1')+"')",'S')
        s=s.replace('len(times)>280','len(times)>1700').replace('59<=(times[-1]-epoch)*1e-9<=60.1','354<=(times[-1]-epoch)*1e-9<=355.12')
        assert str(H83) not in s and str(P83) not in s
    put(P/name,s)
    diff.extend(difflib.unified_diff(original.splitlines(True),s.splitlines(True),fromfile=str(P80/name),tofile=str(P/name)))
for name in ('input_profiles','private_build','transport_tools'):
    shutil.copytree(P80/name,P/name,ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
# Transport protocol pins the wrapper's exact path, not just its model/hash.
protocol=P/'transport_tools/live_receiver_protocol.py'
protocol.write_text(rebase(protocol.read_text()))
for name in ('controller.py','launch.py','evaluate.py','verify_report.py','prepare_host.py'):
    source=H83/name if name in ('controller.py','launch.py','prepare_host.py') else H80/name
    s=rebase(source.read_text()).replace('CP-V80','CP-V84').replace('cpv80_','cpv84_').replace('CP-V83','CP-V84').replace('EIG2-CPV83','EIG2-CPV84')
    if name in ('controller.py','launch.py'):
        s=s.replace('60s','full355.02s').replace('duration_s=60.0','duration_s=355.02')
    if name=='evaluate.py':
        s=s.replace("glim_factor_mode='active_window_temporary'","glim_factor_mode='inject_persistent'")
        s=s.replace('fresh GLIM, general-persistent Kimera','persistent GLIM, general-persistent Kimera')
        s=s.replace('GLIM remains active_window_temporary; CP76 persistent Kimera.','CP82 GLIM inject_persistent AND CP76 persistent Kimera.')
    if name=='verify_report.py':s=s.replace("r['glim_factor_mode']=='active_window_temporary'","r['glim_factor_mode']=='inject_persistent'")
    if name=='prepare_host.py':
        s=s.replace("CP80=E/'"+H.name+"'", "CP80=E/'eig2_persistent_live_cpv80_20261101'")
        s=s.replace('source_bytes*(60.0/355.02)*1.5','source_bytes*1.5').replace('duration_ratio=60.0/355.02','duration_ratio=1.0')
        block="parent_audit=Path(audit['reused_verified_60s_audit'])\nassert read(parent_audit)['window']==audit['window']\npins[str(parent_audit)]=sha(parent_audit)\nassert audit['stereo_pairs']==600 and audit['frontend_min']==597 and audit['frontend_max']==600"
        assert block in s
        s=s.replace(block,"assert audit['passed'] and audit['stereo_pairs']==3550 and audit['frontend_min']==3547 and audit['frontend_max']==3550")
        s=s.replace("audit['window']['duration_s']-60.0","audit['window']['duration_s']-355.02")
        s=s.replace('Single 60s live bidirectional','Single full355.02s live bidirectional')
        needle="pins=read(OLD/'protected_before.json')"
        replacement="""CP83=E/'eig2_symmetric_persistent60_cpv83_20261101'
F83=CP83/'report_completion_1'
assert read(F83/'complete.json')['passed'] and read(F83/'post_closeout.json')['passed']
pins=read(W/'runs'/CP83.name/'prepared/protected_manifest.json')
pins.update(read(F83/'complete.json')['output_sha256'])
pins.update(read(F83/'input_sha256.json'))
for name in ('complete.json','post_closeout.json'):pins[str(F83/name)]=sha(F83/name)
for name in ('failed.json','status.json','saved_validation_repair_1/failed.json'):
    pins[str(CP83/name)]=sha(CP83/name)"""
        assert needle in s;s=s.replace(needle,replacement)
    put(H/name,s)
put(H/'template_changes.diff',''.join(diff))
print(json.dumps(dict(prepared_templates=str(P),source_files=len(files),source_capture_started=False),indent=2))
