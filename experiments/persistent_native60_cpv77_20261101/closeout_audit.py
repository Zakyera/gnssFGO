"""Read-only saved-result/ownership audit. Never invokes an estimator or EVO."""
import csv,datetime,hashlib,json,re,subprocess
from pathlib import Path
H=Path(__file__).resolve().parent;W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3');K=W/'runs'/H.name;P=K/'replay'
def js(p):return json.loads(Path(p).read_text())
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
complete=js(H/'complete.json');assert complete['passed']
for p,v in complete['output_sha256'].items():assert sha(p)==v,p
stamps={};bindings={};decisions={}
for mode in ('temporary','persistent'):
    D=P/(mode+'_capture');v=js(D/'validation.json');assert v['valid']
    stamps[mode]=[int(x['field.header.stamp']) for x in csv.DictReader((D/'kimera_odometry.csv').open())]
    bindings[mode]=[];decisions[mode]={}
    for l in (D/'kimera.log').read_text().splitlines():
        if 'KIMERA_CBS_MEAN_ALIGNMENT_ROW,' in l:
            a=l.split('KIMERA_CBS_MEAN_ALIGNMENT_ROW,',1)[1].split(',')
            bindings[mode].append(list(map(int,a[:10])))
        if 'CBS_ODOM_MATCH_ROW_G2K,' in l:
            a=l.split('CBS_ODOM_MATCH_ROW_G2K,',1)[1].split(',')
            decisions[mode][a[0]]=a[11]
    assert not any(x in (D/'kimera.log').read_text() for x in ('rollback','Recovering','recovery','CPV40_FATAL','process has died'))
states=subprocess.check_output(['docker','inspect','-f','{{.Name}} {{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'],text=True)
assert states=='/cbsms_ws_gpu exited\n/ubuntu-jammy-humble-nvidia exited\n'
owned={str(js(H/'replay_attempt/start.json')['pid']):False}
for f in (H/'replay_attempt').glob('*_process.json'):owned[str(js(f)['pid'])]=False
for p in owned:owned[p]=Path('/proc',p).exists()
assert not any(owned.values()),owned
git={s:subprocess.check_output(['git','-C',str(W/'src'/s),'status','--short'],text=True) for s in ('cbs','Kimera-VIO')}
assert all(not x for x in git.values()),git
r=js(H/'results.json');a=r['diagnostics']['temporary'];b=r['diagnostics']['persistent']
out=dict(passed=True,time=datetime.datetime.now().astimezone().isoformat(),closed_output_hashes_verified=len(complete['output_sha256']),
    identical_all_native_timestamps=stamps['temporary']==stamps['persistent'],
    identical_native_aligned_candidate_ids_updates_endpoint_times=bindings['temporary']==bindings['persistent'],
    identical_native_admission_identities=js(P/'temporary_capture/validation.json')['native_admissions']==js(P/'persistent_capture/validation.json')['native_admissions'],
    unique_sender_last_decision={m:{d:list(v.values()).count(d) for d in sorted(set(v.values()))} for m,v in decisions.items()},
    late_revision_p95_reduction_percent=100*(1-b['late_40_60s']['previous_revision_m']['p95']/a['late_40_60s']['previous_revision_m']['p95']),
    mean_optimize_time_increase_percent=100*(b['timing']['optimize_ms']['mean']/a['timing']['optimize_ms']['mean']-1),
    containers=states,owned_host_jobs_alive=owned,source_worktrees=git,
    interpretation='Native admissions happened to agree; aligned means can evolve differently. Not frozen-measurement isolation or full/live certification.')
with (H/'post_closeout.json').open('x') as f:json.dump(out,f,indent=2)
print(json.dumps(out,indent=2))
