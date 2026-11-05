"""Saved-only full ON and explicitly FAILED OFF-prefix comparison. No replay."""
from pathlib import Path
import argparse,hashlib,importlib.util,json,sys
sys.dont_write_bytecode=True
import numpy as np
H=Path(__file__).resolve().parent
OLD=H.parent/'arc3_velodyne_qcquarter_cpv17_20261026'
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs')
P=W/'gnssfgo_arc3_velodyne_on_cpv18_20261027/prepared'
S=W/'gnssfgo_arc3_velodyne_qcquarter_cpv17_20261026/prepared'
spec=importlib.util.spec_from_file_location('scoring',OLD/'evaluate_first_published.py')
e=importlib.util.module_from_spec(spec);spec.loader.exec_module(e)
a=e.a;b=e.b
PRIMARY=e.PRIMARY
def save(p,v):
 with p.open('x') as f:json.dump(v,f,indent=2)
def load_inputs(root,mode,secondary=True):
 s=root/f'full_{mode}_source';r=root/f'full_{mode}_receiver/receiver'
 paths={'native_glim':s/'glim_odometry.csv','native_kimera':s/'kimera_odometry.csv',
        'fused_online':r/'online_trajectory.tum'}
 if secondary:paths.update(glim_refined=s/'odom_imu.txt',fused_finalized=r/'finalized_trajectory.tum')
 return paths,{role:a.load_curve(path) for role,path in paths.items()}
def reference(pre):
 gt=Path(pre['bag_files']['cpt7_ie_tc']['path'])
 assert e.sha(gt)==pre['bag_files']['cpt7_ie_tc']['sha256']
 ref=a.g.Reference(gt)
 return gt,(ref.times,np.asarray(ref.poses)[:,:3,3],np.asarray(ref.poses)[:,:3,:3])
def common(curves,ref):
 grid=e.make_grid(max(c[0][0] for c in curves),min(c[0][-1] for c in curves))
 valid=a.sample(ref,grid,50_000_000)[2]
 for c in curves:valid &= a.sample(c,grid,500_000_000)[2]
 return grid,valid
def jump_stats(curve,epoch):
 t,p,r=curve;dt=np.diff(t)*1e-9;d=np.linalg.norm(np.diff(p,axis=0),axis=1)
 selected=(dt>0)&(dt<=.5);local=selected&(t[:-1]>=epoch+300_000_000_000)&(t[1:]<=epoch+310_000_000_000)
 return dict(whole_step_m=e.stat(d[selected]),steps_over_1m=int(np.sum(d[selected]>1)),
  around_old_failure_step_m=e.stat(d[local]) if local.any() else None,
  maximum_step_end_relative_s=float((t[1:][selected][np.argmax(d[selected])]-epoch)*1e-9))
def selftest():
 pre=json.loads((S/'input_audit.json').read_text());gt,ref=reference(pre)
 paths,curves=load_inputs(S,'off',False);grid,mask=common(list(curves.values()),ref)
 tests=a.tests();scores={r:e.score(c,ref,grid,mask) for r,c in curves.items()}
 full={r:e.score_saved(c,ref) for r,c in curves.items()}
 # Same curves compared to themselves must be zero improvement; tests exercise
 # the exact failed-prefix geometry before any new capture, without a solver.
 for r in PRIMARY:assert b.improve(scores[r]['ATE_RMSE_m'],scores[r]['ATE_RMSE_m'])==0
 result=dict(passed=True,synthetic_tests=tests,old_OFF_is_failed=True,
  diagnostic_only=True,rows=int(mask.sum()),roles=list(scores),
  alignment_max_m=max(x['alignment_crosscheck_m'] for x in scores.values()),
  original_prefix_full_saved=full,input_sha256={str(p):e.sha(p) for p in paths.values()})
 save(H/'reporter_preflight.json',result);print(json.dumps(dict(passed=True,rows=int(mask.sum()))))
def main():
 out=P/'evaluation_full';assert not out.exists()
 assert (P/'full_on_control/complete.json').exists() and not (P/'full_on_control/failed.txt').exists()
 for p in [P/'full_on_source/complete.marker',P/'full_on_receiver/complete.marker']:assert p.exists(),p
 assert (S/'full_off_control/failed.txt').exists() and not (S/'full_off_control/complete.json').exists()
 pre=json.loads((P/'input_audit.json').read_text());win=pre['window'];epoch=win['start_ns'];duration=win['duration_s']
 gt,ref=reference(pre);paths={};curves={}
 for mode,root,secondary in [('off',S,False),('on',P,True)]:
  pp,cc=load_inputs(root,mode,secondary)
  paths.update({mode+'_'+r:p for r,p in pp.items()});curves.update({mode+'_'+r:c for r,c in cc.items()})
 primary=[m+'_'+r for m in ('off','on') for r in PRIMARY]
 grid,mask=common([curves[k] for k in primary],ref)
 ongrid,onmask=common([curves['on_'+r] for r in PRIMARY],ref)
 results={}
 for key,c in curves.items():
  mode,role=key.split('_',1)
  product=('DELAYED fixed-lag-finalized' if role=='fused_finalized' else
           'refined GLIM; secondary only' if role=='glim_refined' else 'native first-published/ONLINE')
  own=e.make_grid(c[0][0],c[0][-1]);valid=e.on_grid(c,ref,own)[-1]
  results[key]=dict(path=str(paths[key]),sha256=e.sha(paths[key]),product=product,
   status='ON completed' if mode=='on' else 'OFF FAILED PREFIX ONLY',
   full_saved=e.score_saved(c,ref),full_saved_coverage=b.coverage(own,valid,epoch,duration),
   common=e.score(c,ref,grid,mask),common_coverage=b.coverage(grid,e.on_grid(c,ref,grid,mask)[-1],epoch,duration),
   jumps=jump_stats(c,epoch))
  if mode=='on':results[key]['on_only_common']=e.score(c,ref,ongrid,onmask)
 comparison={}
 for role in PRIMARY:
  off=results['off_'+role]['common'];on=results['on_'+role]['common']
  comparison[role]=dict(off_failed_prefix=off,on_same_prefix=on,
   ATE_improvement_percent=b.improve(off['ATE_RMSE_m'],on['ATE_RMSE_m']),
   translation_RPE1_improvement_percent=b.improve(off['translation_RPE_1s_RMSE_m'],on['translation_RPE_1s_RMSE_m']),
   rotation_RPE1_improvement_percent=b.improve(off['rotation_RPE_1s_RMSE_deg'],on['rotation_RPE_1s_RMSE_deg']))
 gaps=[g for g in pre['known_raw_gaps'] if g['gap_s']>1]
 lo=min(g['before_ns'] for g in gaps)-2_000_000_000;hi=max(g['after_ns'] for g in gaps)+2_000_000_000
 gap_counts={}
 for role in PRIMARY:
  c=curves['on_'+role];gg=e.make_grid(lo,hi);valid=e.on_grid(c,ref,gg)[-1]
  gap_counts[role]=dict(queries=len(gg),supported=int(valid.sum()),intervals_100ms=len(a.intervals(valid,1)))
 e.ROOT=P;on_cov=e.covariances('on');e.ROOT=S;off_cov=e.covariances('off')
 cov={'on':on_cov,'off_failed_prefix':off_cov}
 report=dict(passed=True,standalone_ON=True,OFF_complete=False,results=results,off_on_prefix=comparison,
  common_mask=dict(rows=int(mask.sum()),coverage=b.coverage(grid,mask,epoch,duration),
   primary_products=primary,note='ONLY shared failed-OFF prefix; not full matched sequence'),
  on_full_common_mask=b.coverage(ongrid,onmask,epoch,duration),
  reference=dict(path=str(gt),sha256=e.sha(gt),caveat='IE-TC shares CPT7; not independent prism GT'),
  survived_old_OFF_failure=all(c[0][-1]>epoch+306_000_000_000 for k,c in curves.items() if k.startswith('on_')),
  input_gap_cluster=dict(start_ns=lo,end_ns=hi,coverage=gap_counts),covariance=cov,
  first_published_checks={m:b.check_csv(paths[m+'_native_glim']) for m in ('off','on')},
  input_sha256={str(p):e.sha(p) for p in paths.values()},synthetic_tests=a.tests(),
  limitations=['Full-native paths retain gap chords; common paths sum contiguous pieces',
   'No independent ground truth, equal latency, nonlinear stationarity or general robustness certification',
   'Mean-only source covariance alignment approximation unchanged',
   'Refined GLIM and delayed finalized fusion are secondary, not first-published/ONLINE',
   'OFF catastrophic tail retained: common-prefix aggregate metrics must be read with timing/jumps'])
 assert all(e.sha(Path(p))==sha for p,sha in a.HASHES.items())
 out.mkdir();save(out/'summary.json',report)
 np.savez_compressed(out/'common_masks.npz',timestamp_ns=grid,common_valid=mask,on_timestamp_ns=ongrid,on_valid=onmask)
 print(json.dumps(dict(passed=True,off_on_prefix={k:v['ATE_improvement_percent'] for k,v in comparison.items()})))
if __name__=='__main__':
 parser=argparse.ArgumentParser();parser.add_argument('--self-test',action='store_true');args=parser.parse_args()
 if args.self_test:selftest()
 else:main()
