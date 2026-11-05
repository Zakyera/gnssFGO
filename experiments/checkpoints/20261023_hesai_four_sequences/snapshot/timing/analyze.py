"""Saved telemetry only. No estimator, rosbag, container or solver invocation."""
from pathlib import Path
import collections,csv,hashlib,json,math,re,statistics

H=Path(__file__).resolve().parent
E=H.parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
SPECS=[
 ('ETH1','eth1_awake_full_cpk23_20261021','gnssfgo_eth1_awake_full_cpk23_20261021','results.json',1727775007.855396309,346.32),
 ('EIG-2','eig2_full_cpk24_20261021','gnssfgo_eig2_full_cpk24_20261021','results_eof_review.json',1730638816.563546034,355.02),
 ('SPX-3','spx3_stationary_full_cpk28_20261023','gnssfgo_spx3_stationary_full_cpk28_20261023','results.json',1730565899.872888651,197.02),
 ('LEICA-2','leica2_full_cpk29_20261023','gnssfgo_leica2_full_cpk29_20261023','results.json',1732549034.915071665,428.75)]
ANSI=re.compile(r'\x1b\[[0-9;]*m')
inputs={}
def record(p):
 p=Path(p).resolve()
 if str(p) not in inputs:
  before=p.stat()
  with p.open('rb') as f:sha=hashlib.file_digest(f,'sha256').hexdigest()
  assert (before.st_size,before.st_mtime_ns)==(p.stat().st_size,p.stat().st_mtime_ns)
  inputs[str(p)]=dict(bytes=before.st_size,mtime_ns=before.st_mtime_ns,sha256=sha)
 return p
def load(p):return json.loads(record(p).read_text())
def stat(x):
 x=sorted(float(v) for v in x)
 if not x:return dict(n=0)
 assert all(map(math.isfinite,x))
 return dict(n=len(x),mean=statistics.mean(x),median=statistics.median(x),
             p95=x[math.ceil(len(x)*.95)-1],p99=x[math.ceil(len(x)*.99)-1],max=x[-1],total=sum(x))
def fields(line,marker):return ANSI.sub('',line).split(marker,1)[1].strip().split(',')
def groups(rows,start,duration,key=0):
 return {'whole':rows,'steady':[r for r in rows if r[key]>=start+5],
 'first20_steady':[r for r in rows if start+5<=r[key]<start+25],
 'last20':[r for r in rows if r[key]>=start+duration-20]}
def workload(events,start,duration):
 """Each pair is (physical/record timestamp seconds, measured wall service ms).
 Fixed-service FIFO estimate only, not a replay of the algorithm or hardware.
 """
 events=sorted(events)
 costs=stat(v for t,v in events)
 result={'service_ms':costs,'measured_wall_seconds':costs.get('total',0)/1000,
   'average_busy_fraction_at_020':costs.get('total',0)/1000/duration*.2,
   'average_busy_fraction_at_050':costs.get('total',0)/1000/duration*.5,
   'single_lane_average_saturation_rate':duration/(costs.get('total',1)/1000)}
 for width in (1,5,20):
  left=0;total=0.;best=0.;when=None
  for right,(t,c) in enumerate(events):
   total+=c
   while left<=right and events[left][0]<=t-width:
    total-=events[left][1];left+=1
   if start+width<=t<=start+duration and total>best:best=total;when=t-start
  result[f'peak_{width}s_busy_fraction_at_050']=best/1000/(width/.5)
  result[f'peak_{width}s_window_end_relative_s']=when
 finish=-float('inf');wait=[];lag=[]
 for t,c in events:
  arrival=(t-start)/.5;begin=max(finish,arrival)
  wait.append(max(0,begin-arrival));finish=begin+c/1000;lag.append(finish-arrival)
 result['fixed_service_fifo_at_050']={'wait_wall_s':stat(wait),'completion_wall_s':stat(lag)}
 return result
G_NAMES=['stamp','id','points','imu','new_factors','active_frames','retired_frames',
 'state_lookup_ms','inter_scan_imu_ms','imu_factor_ms','intra_scan_imu_ms','deskew_ms',
 'point_covariance_ms','cpu_frame_ms','create_gpu_frame_ms','create_factors_ms','pre_callback_ms',
 'main_smoother_ms','post_callback_ms','frame_retirement_ms','update_frames_ms','imu_validation_ms',
 'update_callbacks_ms','scan_ms']
K_NAMES=['id','optimize_ms','factor_preparation_ms','receive_beliefs_ms','delete_sort_ms','smoother_ms',
 'slot_bookkeeping_ms','extra_iterations_ms','update_states_ms','outgoing_ms','set_marginal_graph_ms',
 'first_belief_query_ms','state_covariance_ms','debug_ms','published','graph_factors','ok']

out={}
for seq,exp,run,result_name,start,duration in SPECS:
 saved=load(E/exp/result_name);assert saved['passed']
 P=W/'runs'/run/'prepared'
 for mode in ('off','on'):
  S=P/f'full_{mode}_source';R=P/f'full_{mode}_receiver';C=P/f'full_{mode}_control'
  sc=load(S/'source_coverage.json');assert sc['passed']
  effective=load(S/'effective_mode_check.json');assert effective['passed'] and effective['playback_rate']==.2
  logs=sorted({p.resolve() for p in S.glob('ros_logs/*/glim_rosnode-*-stdout.log')});assert len(logs)==1
  scans=[];asyncs=[];finish={};first={};inputs_ros=[];publish_odom={};pre={}
  for line in record(logs[0]).open():
   if 'GLIM_ODOM_IMU_TIMING_ROW,' in line:
    f=fields(line,'GLIM_ODOM_IMU_TIMING_ROW,');assert len(f)==25
    scans.append(list(map(float,f[:24])))
   elif 'GLIM_ASYNC_ODOM_TIMING_ROW,' in line:
    f=fields(line,'GLIM_ASYNC_ODOM_TIMING_ROW,')
    if f[-1]=='processed':asyncs.append(list(map(float,f[:-1])))
   elif 'GLIM_CBS_TIMING_ROW,on_smoother_update_finish,' in line:
    f=fields(line,'GLIM_CBS_TIMING_ROW,');finish[int(f[1])]=list(map(float,f[1:]))
   elif 'GLIM_CBS_TIMING_ROW,on_smoother_update,' in line:
    f=fields(line,'GLIM_CBS_TIMING_ROW,');pre[int(f[1])]=list(map(float,f[1:]))
   elif 'GLIM_CBS_TIMING_ROW,publish_outgoing,' in line:
    f=fields(line,'GLIM_CBS_TIMING_ROW,');first[int(f[1])]=float(f[16])
   elif 'GLIM_CBS_TIMING_ROW,publish_odometry,' in line:
    f=fields(line,'GLIM_CBS_TIMING_ROW,');publish_odom[int(f[1])]=float(f[12])
   elif 'GLIM_ROS_INPUT_TIMING_ROW,' in line:
    f=fields(line,'GLIM_ROS_INPUT_TIMING_ROW,');inputs_ros.append(float(f[7]))
  assert len(scans)==sc['native_glim']['rows'],(seq,mode,len(scans),sc['native_glim']['rows'])
  G={}
  for cohort,rows in groups(scans,start,duration).items():
   stages={name:stat(r[i] for r in rows) for i,name in enumerate(G_NAMES) if i>=7}
   sumscan=stages['scan_ms']['total']
   stages['additive_shares_pct']={name:100*v['total']/sumscan for name,v in stages.items() if name!='scan_ms'}
   stages['unassigned_ms']=sumscan-sum(v['total'] for k,v in stages.items() if k.endswith('_ms') and k!='scan_ms')
   callback=[finish[int(r[1])][12] for r in rows if int(r[1]) in finish]
   revision_extra=[finish[int(r[1])][12]-first.get(int(r[1]),0) for r in rows if int(r[1]) in finish]
   stages['outgoing_plus_revisions_ms']=stat(callback)
   stages['revision_plus_activepose_remainder_ms']=stat(revision_extra)
   stages['first_outgoing_ms']=stat(first[int(r[1])] for r in rows if int(r[1]) in first)
   stages['pre_callback_substages_ms']={name:stat(pre[int(r[1])][index] for r in rows if int(r[1]) in pre)
       for name,index in [('timestamp_and_prune',7),('consume_incoming',8),('inject_loop',9),('local_sidecar_admission',10),('remove_expired_factors',11)]}
   stages['over_200ms']=sum(r[23]>200 for r in rows)
   stages['over_500ms']=sum(r[23]>500 for r in rows)
   G[cohort]=stages
  valid_asyncs=[r for r in asyncs if start-1<=r[0]<=start+duration]
  G['workload']=workload([(max(r[0],r[1]),r[7]) for r in valid_asyncs],start,duration)
  G['startup_async_zero_timestamps_excluded_from_projection']=len(asyncs)-len(valid_asyncs)
  G['input_preprocess_ms']=stat(inputs_ros)
  G['internal_queue_max']=max(r[9] for r in asyncs)
  G['max_active_frames']=max(r[5] for r in scans)
  G['top_slow_scans']=[dict(relative_s=r[0]-start,pre_components=pre.get(int(r[1])),**{name:r[i] for i,name in enumerate(G_NAMES) if i>=7}) for r in sorted(scans,key=lambda r:r[23],reverse=True)[:8]]
  owner=collections.defaultdict(lambda:collections.Counter());worker=collections.defaultdict(lambda:collections.Counter());nctx=collections.Counter();sync=0;hits=0
  for line in record(S/'e1b_profile.jsonl').open():
   r=json.loads(line);assert not r['aborted'];ctx=r['context'];nctx[ctx]+=1
   for k,v in r['owner_wall'].items():owner[ctx][k]+=v[2]/1e6
   for k,v in r['worker_sum'].items():worker[ctx][k]+=v[2]/1e6
   if ctx==1:
    hits+=r['metrics'][7];sync+=r['owner_wall'].get('gpu_sync_linearize',[0])[0]
  G['profile']={'sessions':dict(nctx),'owner_exclusive_ms':dict(owner),'worker_sum_exclusive_ms':dict(worker),'main_gpu_prepared_hits':hits,'main_gpu_sync_calls':sync}

  with record(S/'kimera_odometry.csv').open() as f:poses=list(csv.DictReader(f))
  stamps={i:int(r['field.header.stamp'])*1e-9 for i,r in enumerate(poses)}
  tables=collections.defaultdict(dict);frontend_last='';frontend_samples={};seen=set()
  kimlogs=[S/'estimators.log',*sorted({p.resolve() for p in S.glob('ros_logs/*/kimera_vio_ros-kimera_vio_ros_node-*-stdout.log')})]
  for log in kimlogs:
   for line in record(log).open():
    if 'VioFrontend [ms]' in line:
     frontend_last=line.strip();parts=line.split('\t')
     if len(parts)>=6:
      frontend_samples[int(parts[1])]=float(parts[-1])
    if 'KIMERA_' not in line or 'TIMING_ROW,' not in line:continue
    payload=ANSI.sub('',line).split('KIMERA_',1)[1].strip()
    if payload in seen:continue
    seen.add(payload)
    marker,payload=payload.split(',',1)
    if marker not in ['OPTIMIZE_TIMING_ROW','BACKEND_SPINONCE_TIMING_ROW','BACKEND_CALLBACK_TIMING_ROW']:continue
    row=list(map(float,payload.split(',')));index=int(row[0]);assert index in stamps
    assert index not in tables[marker],(seq,mode,marker,index)
    tables[marker][index]=row
  opt=tables['OPTIMIZE_TIMING_ROW'];back=tables['BACKEND_SPINONCE_TIMING_ROW'];callback=tables['BACKEND_CALLBACK_TIMING_ROW']
  assert len(back)==len(poses) and len(opt)==len(poses)
  K={}
  for cohort,rows in groups([[stamps[i],i] for i in back],start,duration).items():
   ids=[r[1] for r in rows]
   v={name:stat(opt[i][j] for i in ids) for j,name in enumerate(K_NAMES) if 1<=j<=13}
   v['backend_ms']=stat(back[i][1] for i in ids)
   v['callback_ms']=stat(callback[i][1] for i in ids if i in callback)
   v['first_query_setup_plus_query_ms']=stat(opt[i][10]+opt[i][11] for i in ids)
   v['revision_and_other_outgoing_ms']=stat(opt[i][9]-opt[i][10]-opt[i][11] for i in ids)
   v['smoother_share_of_backend_pct']=100*v['smoother_ms']['total']/v['backend_ms']['total']
   v['outgoing_share_of_backend_pct']=100*v['outgoing_ms']['total']/v['backend_ms']['total']
   v['over_400ms']=sum(back[i][1]>400 for i in ids)
   K[cohort]=v
  K['workload']=workload([(stamps[i],back[i][1]+callback.get(i,[0,0])[1]) for i in back],start,duration)
  K['frontend_last_statistics_line']=frontend_last
  parts=frontend_last.split('\t');count=int(parts[1]);mean=float(parts[3].strip().split()[0].lstrip('{'))
  minimum,maximum=map(float,parts[4].strip().strip('[]').split(','))
  assert count==sc['kimera_processing']['frontend_processed']
  K['frontend_summary']={'n':count,'rounded_cumulative_mean_ms':mean,'max_ms':maximum,'average_busy_fraction_at_050':count*mean/1000/duration*.5}
  K['frontend_sampled_last_value_ms']=stat(frontend_samples.values())
  K['keyframe_interval_s']=stat(b-a for a,b in zip(stamps.values(),list(stamps.values())[1:]))

  events=[];supports=[];ticks=[];bykind=collections.defaultdict(list);publications=0;bucket=0.
  with record(R/'receiver/stream_progress.csv').open() as f:
   for row in csv.DictReader(f):
    t=int(row['arrival_ns'])*1e-9;ms=float(row['service_seconds'])*1000
    bykind[row['kind']].append(ms)
    bucket+=ms
    if row['kind']!='E':events.append((t,ms))
    pubs=int(row['publications'])
    if pubs>publications:
     assert pubs==publications+1
     supports.append([t,bucket,1]);ticks.append(ms);bucket=0.;publications=pubs
  F={'service_by_kind_ms':{k:stat(v) for k,v in bykind.items()},'workload':workload(events,start,duration)}
  F['publishing_clock_event_only_ms']=stat(ticks)
  for cohort,rows in groups(supports,start,duration).items():F[cohort]={'support_service_ms':stat(r[1] for r in rows),'over_200ms':sum(r[1]>200 for r in rows)}
  # These are all-event serial costs, NOT just the support-cycle optimizer.
  F['publishing_clock_event_cost_fraction_pct']=100*sum(ticks)/sum(v for t,v in events)
  F['belief_event_cost_fraction_pct']=100*sum(bykind['B'])/sum(v for t,v in events)
  expected=saved['cases'][mode]['checks']['receiver']['support_cycle_seconds']
  assert abs(F['whole']['support_service_ms']['mean']/1000-expected['mean'])<1e-9
  assert abs(F['whole']['support_service_ms']['median']/1000-expected['median'])<1e-9
  F['unsupported_substage_claim']='No separate receiver optimizer/marginalization/I/O timers in these logs.'
  mon=[json.loads(line) for line in record(C/'monitor.jsonl').open()]
  F['monitor_age_s']=stat(r['output_age_seconds'] for r in mon if r.get('output_age_seconds') is not None and r['published_poses']>0)
  F['monitor_queue_max']=max(r['queued'] for r in mon)
  F['transport_max_queue']=max(r['max_queue'] for r in mon)
  F['retained_supports']=saved['cases'][mode]['checks']['receiver'].get('retained_supports')
  out[seq+' '+mode]={'sequence':seq,'mode':mode,'duration_s':duration,'source':str(P),
   'glim':G,'kimera':K,'fusion':F,'capture':load(C/'complete.json')}
  print(seq,mode,'scan/backend/support median p95 max',*[tuple(round(v[k],2) for k in ('median','p95','max')) for v in [G['steady']['scan_ms'],K['steady']['backend_ms'],F['steady']['support_service_ms']]],flush=True)
  print(' .5x average/peak20s utilization',*[(n,round(d['workload']['average_busy_fraction_at_050'],3),round(d['workload']['peak_20s_busy_fraction_at_050'],3)) for n,d in [('G',G),('K',K),('F',F)]],flush=True)
for path,info in inputs.items():
 st=Path(path).stat();assert (st.st_size,st.st_mtime_ns)==(info['bytes'],info['mtime_ns'])
with (H/'timing_v2.json').open('x') as f:json.dump({'cases':out,'inputs':inputs,'method':'saved wall-service only, nearest-rank percentiles; no hardware/rate/solver replay; cycles include all belief+clock work between publications'},f,indent=2)
