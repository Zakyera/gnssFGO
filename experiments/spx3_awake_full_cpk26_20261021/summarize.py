"""Read existing scored products and ordinary telemetry; no optimizer invocation."""
from pathlib import Path
import collections,csv,hashlib,json,math,re,statistics

H=Path(__file__).resolve().parent
P=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_spx3_awake_full_cpk26_20261021/prepared')
V=Path('/home/yeranis/results/gnssfgo_spx3_awake_full_cpk26_20261021')
def load(p):return json.loads(p.read_text())
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def stat(values):
 a=sorted(map(float,values));assert a and all(map(math.isfinite,a))
 return dict(n=len(a),median=statistics.median(a),p95=a[math.ceil(.95*len(a))-1],max=a[-1])
def fields(line,marker):
 return re.sub(r'\x1b\[[0-9;]*m','',line).split(marker,1)[1].strip().split(',')

assert not (H/'results.json').exists()
manifest=load(P/'protected_manifest.json')
for p,w in manifest.items():assert sha(p)==w,p
evaluation=load(P/'evaluation_full/summary.json')
cases={}
for mode in ('off','on'):
 S=P/f'full_{mode}_source';R=P/f'full_{mode}_receiver';C=P/f'full_{mode}_control'
 assert not (C/'failed.txt').exists()
 checks={k:load(p) for k,p in dict(receiver=V/mode/'validation.json',
     lifecycle=S/'candidate_validation.json',local=S/'local_provenance.json').items()}
 assert all(v['passed'] for v in checks.values())
 effective=load(S/'effective_mode_check.json')
 assert effective['cached_provider_sha256']=='6f71292d29d974a0555abd9fda6b2370fdab81ca634a4e1343c216d2a762d2ca'
 glim_times=[float(l.split()[0]) for l in (S/'odom_imu.txt').open() if l.strip()]
 start=1730565902.872888651;last=max(glim_times)
 logs=list(S.glob('ros_logs/*/glim_rosnode-*-stdout.log'));assert len(logs)==1
 scans=[];queues=[]
 for line in logs[0].open():
  if 'GLIM_ODOM_IMU_TIMING_ROW,' in line:
   f=fields(line,'GLIM_ODOM_IMU_TIMING_ROW,');assert len(f)==25
   if float(f[0])<=last+1e-7:scans.append(list(map(float,f[:24])))
  if 'GLIM_ASYNC_ODOM_TIMING_ROW,' in line:
   f=fields(line,'GLIM_ASYNC_ODOM_TIMING_ROW,')
   if f[-1]=='processed':queues.append(int(f[9]))
 cohorts={}
 for label,begin in [('whole',0),('after5s',5),('last20s',174.02)]:
  selected=[r for r in scans if start+begin<=r[0]<=last+1e-7]
  cohorts[label]={name:stat(r[col] for r in selected) for name,col in
      [('scan_ms',23),('main_ms',17),('post_callback_ms',18)]}
  cohorts[label]['scans_over_500ms']=sum(r[23]>500 for r in selected)
 main=[r for r in map(json.loads,(S/'e1b_profile.jsonl').open()) if r['context']==1 and r['timestamp']<=last+1e-7]
 assert main and all(not r['aborted'] for r in main)
 work=collections.Counter()
 for r in main:
  for k,v in r['owner_wall'].items():work[k]+=v[0]
 hits=sum(r['metrics'][7] for r in main)
 assert hits==work['gpu_linearize']>0 and work['gpu_sync_linearize']==0
 with (S/'kimera_odometry.csv').open() as f:poses=list(csv.DictReader(f))
 ids={i for i,r in enumerate(poses) if int(r['field.header.stamp'])*1e-9>=start+5}
 kimera=collections.defaultdict(list)
 kimera_logs=[S/'estimators.log',*S.glob('ros_logs/*/kimera_vio_ros-kimera_vio_ros_node-*-stdout.log')]
 seen_timing=set()
 for line in (line for log in kimera_logs for line in log.open()):
  timing_payload=line.split('KIMERA_',1)[-1].strip()
  if timing_payload in seen_timing:continue
  if 'KIMERA_' in line:seen_timing.add(timing_payload)
  if 'KIMERA_BACKEND_SPINONCE_TIMING_ROW,' in line:
   f=fields(line,'KIMERA_BACKEND_SPINONCE_TIMING_ROW,')
   if int(f[0]) in ids:kimera['backend_ms'].append(float(f[1]))
  if 'KIMERA_OPTIMIZE_TIMING_ROW,' in line:
   f=fields(line,'KIMERA_OPTIMIZE_TIMING_ROW,')
   if int(f[0]) in ids:
    for k,i in [('optimize_ms',1),('smoother_ms',5),('outgoing_ms',9)]:kimera[k].append(float(f[i]))
 cases[mode]=dict(checks=checks,capture=load(C/'complete.json'),
     glim_timing=cohorts,glim_internal_queue_max=max(queues),
     main_prepared_hits=hits,main_sync_calls=work['gpu_sync_linearize'],
     kimera_timing_after5s={k:stat(v) for k,v in kimera.items()},
     source_coverage=load(S/'source_coverage.json'))

for folder in ('glim_config','kimera_profile'):
 trees=[]
 for mode in ('off','on'):
  d=P/f'full_{mode}_source'/folder
  trees.append({str(p.relative_to(d)):sha(p) for p in d.rglob('*') if p.is_file()})
 assert trees[0]==trees[1],folder
result=dict(passed=True,validation_contract='finite_recording_eof_cpk24a',terminal_omissions={m:c['checks']['receiver']['terminal_boundary']['excluded_versions'] for m,c in cases.items()},cases=cases,evaluation=evaluation,protected_unchanged=len(manifest),
    identical_source_profiles=True,no_exhaustive_observer=True,no_saved_journal_repeat=True,
    limits='One full-sequence experimental pair, not an exhaustive batching-preservation certificate; IE-TC shares CPT7; no nonlinear stationarity claim.')
with (H/'results.json').open('x') as f:json.dump(result,f,indent=2)
lines=['CP-K26 matched SPX-3 batched finalized-policy full194.02s OFF/ON — COMPLETED',
 'One capture per mode,0.20x,headless. No paired observer,optimizer rebuild or tuning.',
 'Native GLIM / native Kimera ROS odometry / concurrent ONLINE fusion.',
 'EOF contract: only final partial-cell omissions allowed; every omission disclosed below. No fabricated support.',
 'Terminal omissions OFF/ON: '+str(result['terminal_omissions']),
 'Reference: full-window IE-TC shares CPT7; not independent prism GT.',
 'Existing rigid no-scale ATE alignment and prism lever arm;1sRPE unchanged.','',
 'FULL-SAVED OUTPUTS: every saved pose against reference; NOT a favorable short mask.',
 'role mode ATE_m tRPE1s_m rRPE1s_deg path_prism_m span_s rows']
for role in ('native_glim','native_kimera','fused_online','fused_finalized'):
 for mode in ('off','on'):
  r=evaluation['results'][mode+'_'+role]['full_saved']
  lines.append(f"{role} {mode} {r['ATE_RMSE_m']:.6f} {r['translation_RPE_1s_RMSE_m']:.6f} {r['rotation_RPE_1s_RMSE_deg']:.6f} {r['path_length_prism_m']:.6f} {r['span_s']:.6f} {r['rows']}")
lines+=['','COMMON MASK: all sources/modes, same physical timestamps.',json.dumps(evaluation['common_mask']),
 'role mode ATE_m tRPE1s_m rRPE1s_deg path_prism_m']
for role in ('native_glim','native_kimera','fused_online','fused_finalized'):
 for mode in ('off','on'):
  r=evaluation['results'][mode+'_'+role]['common']
  lines.append(f"{role} {mode} {r['ATE_RMSE_m']:.6f} {r['translation_RPE_1s_RMSE_m']:.6f} {r['rotation_RPE_1s_RMSE_deg']:.6f} {r['path_length_prism_m']:.6f}")
lines+=['','COVERAGE / TIMING / CONSERVATION']
for mode,c in cases.items():
 lines+=[mode,json.dumps(c,indent=2)]
lines+=['','SOURCE COVARIANCE SUMMARY: unchanged dense6x6 scales1; rotation then translation.',
 json.dumps(evaluation['covariance'],indent=2),'',
 'LIMITS: DELAYED finalized is not ONLINE or full-history. Startup/tail gaps are',
 'reported in source_coverage, including sparse stationary-tail keyframes. Common',
 'mask segments exclude unsupported interpolation; its envelope is not dense coverage.',
 'Mean-only source-to-source alignment covariance approximation unchanged.',
 'Native sensors both still use CPT7; no central IMU/GNSS factor in7s WNOA receiver.',
 'Stable-ID downstream revisions replace rather than accumulate. Source CBS temporary/classic.',
 'No saved-journal replay or exhaustive same-input checker requested/run. Existing K16',
 'incomplete preservation gates remain incomplete; no default promotion or exhaustive preservation claim.',
 'Known post-absorption revisions are explicitly expired under CP-K19; counts and',
 'forgone mean/covariance changes are included in checks.receiver.expiry above.',
 'These changes are not GT errors; the late corrections were not incorporated.',
 'Raw source means,covariances,revisions,events,profiles,hashes and timing retained under:',str(P)]
with (H/'RESULTS_REPORT.txt').open('x') as f:f.write('\n'.join(lines)+'\n')
print('\n'.join(lines[:14]))
