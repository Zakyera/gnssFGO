"""Mechanically extend the verified CP77 harness into fresh full-sequence roots."""
import ast,hashlib,json,shutil,xml.etree.ElementTree as ET
from pathlib import Path
H=Path(__file__).resolve().parent;E=H.parent;W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
OLD=E/'persistent_native60_cpv77_20261101';KOLD=W/'runs'/OLD.name
K=W/'runs'/H.name;R=K/'replay';assert not K.exists();R.mkdir(parents=True)
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def rep(s,a,b):
    assert a in s,a
    return s.replace(a,b)
assert json.loads((OLD/'complete.json').read_text())['passed']
assert json.loads((OLD/'post_closeout.json').read_text())['passed']
shutil.copytree(KOLD/'private_build',K/'private_build');shutil.copyfile(KOLD/'binary_gate.json',K/'binary_gate.json')
shutil.copytree(KOLD/'replay/kimera_profile',R/'kimera_profile')
for p in (KOLD/'replay').glob('*.launch'):
    shutil.copyfile(p,R/p.name);ET.parse(R/p.name)
s=(KOLD/'replay/run_case.py').read_text().replace('11577','11578').replace('60.0','355.02').replace('+480','+2100')
s=rep(s,"'active_window_rollback','CPV40_FATAL'","'active_window_rollback','rollback_and_retry_without_staged_cbs','rollback_remove_active_and_retry','CPV40_FATAL'")
(R/'run_case.py').write_text(s)
s=(KOLD/'replay/prepare_inputs.py').read_text()
s=rep(s,'60_000_000_000','355_020_000_000').replace('duration=60.','duration=355.02')
s=rep(s,"assert len(beliefs)>500,'unexpectedly incomplete raw stream'","assert len(beliefs)==len(rows)==total==3541,('full raw stream count/window mismatch',len(beliefs),len(rows),total)")
(R/'prepare_inputs.py').write_text(s)
s=(KOLD/'replay/validate.py').read_text()
s=rep(s,"'process has died','terminate called'","'rollback_and_retry_without_staged_cbs','rollback_remove_active_and_retry','process has died','terminate called'")
s=rep(s,'len(poses)>290','len(poses)>1700')
s=rep(s,'59<=(poses[-1]-epoch)*1e-9<=60.1','354<=(poses[-1]-epoch)*1e-9<=355.12\nassert max(np.diff(poses))<=1_100_000_000,\'coverage gap exceeds predeclared1.1s\'')
s=s.replace('>250','>1500')
s=rep(s,'align=[];bounds=[];transport=[];adds=[];decisions=collections.Counter()','align=[];bounds=[];transport=[];adds=[];decisions=collections.Counter();last_decision={}')
s=rep(s,"decisions[v[11]]+=1","decisions[v[11]]+=1;last_decision[tuple(map(int,re.findall(r'\\d+',v[0])))]=v[11]")
s=rep(s,'native_decision_counts=dict(decisions),native_admissions=adds,','native_decision_counts=dict(decisions),native_admissions=adds,\n    unique_last_decision_counts=dict(collections.Counter(last_decision.values())),\n    received_sender_ids_without_match_row=[list(k) for k in wire if k not in last_decision],')
(R/'validate.py').write_text(s)
for name in ('evaluate.py','verify_report.py','summarize.py','prepare_host.py','controller.py'):
    s=(OLD/name).read_text().replace('CP-V77','CP-V78')
    if name=='evaluate.py':
        s=s.replace('Kimera-only60s','Kimera-only355.02s').replace('epoch,60.)','epoch,355.02)')
        s=rep(s,"late=[e for e in events if 40<=e['time_s']<=60]","late=[e for e in events if 40<=e['time_s']<=60]\n    tail=[e for e in events if 295.02<=e['time_s']<=355.02]")
        s=rep(s,'cost=dict(rows=len(cost)',"tail_295_355s={k:stats([e[k] for e in tail]) for k in ('previous_revision_m','same_update_motion_m','absolute_step_m','revision_rotation_deg')},\n        cost=dict(rows=len(cost)")
    elif name=='summarize.py':
        s=s.replace('HESAI,60s','HESAI,FULL355.02s').replace('No default/full-mission promotion.','No default or live-bidirectional promotion.')
        s=rep(s,"'Native decisions (events, includes retries): '+json.dumps(v['native_decision_counts']),","'Native decisions (events, includes retries): '+json.dumps(v['native_decision_counts']),\n        'Unique last decisions: '+json.dumps(v['unique_last_decision_counts']),\n        'Received sender IDs without match row: '+json.dumps(v['received_sender_ids_without_match_row']),")
        s=rep(s,"'Costs: '+json.dumps(d['cost'])","'Final60s previous-pose revisions: '+json.dumps(d['tail_295_355s']['previous_revision_m']),\n        'Costs: '+json.dumps(d['cost'])")
        s=rep(s,"'General persistent changes admission/replacement as well as retirement: not a retention-only causal ablation.',","'General persistent changes admission/replacement as well as retirement: not a retention-only causal ablation.',\n 'Different total graph objectives are not directly comparable across these policies.',")
    elif name=='prepare_host.py':
        s=rep(s,"pins.update(json.loads((E/'retention_readiness_cpv73_20261031/complete.json').read_text())['output_sha256'])","pins.update(json.loads((E/'retention_readiness_cpv73_20261031/complete.json').read_text())['output_sha256'])\nold=E/'persistent_native60_cpv77_20261101'\npins.update(json.loads((old/'protected_prelaunch.json').read_text()))\npins.update(json.loads((old/'complete.json').read_text())['output_sha256'])\nfor p in (old/'complete.json',old/'INTERPRETATION.txt',old/'post_closeout.json',old/'closeout_audit.py'):pins[str(p)]=sha(p)")
        s=s.replace("raw['selected_beliefs']>500","raw['selected_beliefs']==3541").replace('persistent 60s','persistent full355.02s')
    elif name=='controller.py':
        s=s.replace('Finite60s','Finite355.02s').replace("H/'protected_prelaunch.json'","H/'protected_before.json'")
        s=s.replace("raw['selected_beliefs']>500 and raw['duration']==60.","raw['selected_beliefs']==3541 and raw['duration']==355.02")
        s=rep(s,"cleanup_error=cleanup_error,\n    integrity_error=integrity_error", "cleanup_error=cleanup_error,\n    integrity_error=integrity_error")
        s=rep(s,"passed=scored and not any((failure,cleanup_error,integrity_error))", "owned_jobs={str(json.loads(p.read_text())['pid']):False for p in D.glob('*_process.json')}\nfor pid in owned_jobs:owned_jobs[pid]=Path('/proc',pid).exists()\nif any(owned_jobs.values()):integrity_error='owned stage still alive: '+str(owned_jobs)\npassed=scored and not any((failure,cleanup_error,integrity_error))")
        s=rep(s,'completed=now(),controller_pid=os.getpid(),controller_exiting=True,','completed=now(),controller_pid=os.getpid(),controller_exiting=True,owned_stage_jobs_alive=owned_jobs,')
    ast.parse(s);(H/name).write_text(s)
s=(E/'retention_readiness_cpv73_20261031/launch.py').read_text().replace('CP-V73','CP-V78').replace('readiness-gated retention','native-matching persistent')
ast.parse(s);(H/'launch.py').write_text(s)
for p in R.glob('*.py'):ast.parse(p.read_text())
print('Fresh CP78 harness prepared; no build or replay launched.')
