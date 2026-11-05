"""One saved-data decision report; shared grid, no source rerun or tuning."""
from pathlib import Path
import importlib.util,json,sys
sys.dont_write_bytecode=True
import numpy as np
H=Path(__file__).resolve().parent
P=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_leica2_gravity_projection_cpv9_20261026/prepared')
spec=importlib.util.spec_from_file_location('geometry',H.parent/'leica2_drift_attribution_cpv7_20261025/analyze.py')
a=importlib.util.module_from_spec(spec);spec.loader.exec_module(a)
def rms(v):return float(np.sqrt(np.mean(np.asarray(v)**2)))
def local(p):return Path(p.replace('/workspace/cbs_gtsam4.3','/home/yeranis/repos/V4RL/cbs_gtsam4.3'))
def support(valid):
 ids=np.flatnonzero(valid);pieces=np.split(ids,np.flatnonzero(np.diff(ids)>1)+1)
 return sum(max(0,len(q)-1)*.1 for q in pieces),pieces
def main():
 assert not (H/'decision.json').exists()
 record=json.loads(a.pin(H/'results.json').read_text());assert record['passed']
 ev=record['evaluation'];modes=('off','baseline_on','on');roles=('native_glim','native_kimera','fused_online')
 curves={m+'_'+r:a.load_curve(local(ev['results'][m+'_'+r]['path'])) for m in modes for r in roles}
 ref=a.g.Reference(a.pin(local(ev['reference']['path'])))
 reference=ref.times,np.array(ref.poses)[:,:3,3],np.array(ref.poses)[:,:3,:3]
 base=[v for k,v in curves.items() if not k.startswith('on_')]
 start=max(c[0][0] for c in base);end=min(c[0][-1] for c in base)
 grid=np.arange(((start+a.STEP-1)//a.STEP)*a.STEP,(end//a.STEP)*a.STEP+1,a.STEP,dtype=np.int64)
 gp,gr,valid=a.sample(reference,grid,50_000_000);basevalid=valid.copy();samples={}
 for name,c in curves.items():
  p,r,ok=a.sample(c,grid,500_000_000);samples[name]=p,r;valid&=ok
  if not name.startswith('on_'):basevalid&=ok
 fullsupport,_=support(basevalid);secs,pieces=support(valid);ids=np.flatnonzero(valid);assert len(ids)>500
 ppgt,prgt=a.prism(gp,gr);metrics={};arrays=dict(timestamp_ns=grid,valid=valid,time_s=(grid-a.START)*1e-9)
 for name,(p,r) in samples.items():
  pp,pr=a.prism(p,r);d=dict(common_ATE_m=rms(np.linalg.norm(a.best_fit(pp[valid],ppgt[valid]),axis=1)),horizons={},anchors={})
  for h in (1,10,20):
   i=a.intervals(valid,h*10);j=i+h*10
   dt,dr,_=a.rpe(pp,pr,ppgt,prgt,i,j)
   _,_,world=a.rpe(p,r,gp,gr,i,j)
   d['horizons'][str(h)]=dict(n=len(i),translation_RMS_m=rms(np.linalg.norm(dt,axis=1)),rotation_RMS_deg=rms(np.linalg.norm(dr,axis=1)),gravity_axis_rotation_RMS_deg=rms(world[:,2]))
  for shift in (0,1,5):
   anchor=int(ids[0])+shift*10;assert valid[anchor],('unsupported anchor',shift)
   later=valid&(np.arange(len(grid))>=anchor)
   dp,_,_=a.fixed_start(pp,pr,ppgt,prgt,anchor)
   _,_,angles=a.fixed_start(p,r,gp,gr,anchor)
   d['anchors'][str(shift)]=dict(timestamp_ns=int(grid[anchor]),heading_RMS_deg=rms(angles[later,2]),terminal_heading_deg=float(angles[ids[-1],2]),terminal_position_m=float(np.linalg.norm(dp[ids[-1]])))
   if shift==0:arrays[name+'_heading_deg']=angles[:,2];arrays[name+'_position_error_m']=np.linalg.norm(dp,axis=1)
  d['supported_path_m']=float(sum(np.linalg.norm(np.diff(pp[q],axis=0),axis=1).sum() for q in pieces))
  c=curves[name];fp,fr,ok=a.sample(reference,c[0],50_000_000);assert ok.all()
  fep,_=a.prism(c[1],c[2]);fgp,_=a.prism(fp,fr)
  d['full_ATE_reproduction_error_m']=abs(rms(np.linalg.norm(a.best_fit(fep,fgp),axis=1))-ev['results'][name]['full_saved']['ATE_RMSE_m'])
  assert d['full_ATE_reproduction_error_m']<1e-5
  metrics[name]=d
 checks={}
 def ratio(label,c,b,limit):checks[label]=dict(passed=bool(c<=limit*b+1e-10),candidate=c,original_on=b,maximum_ratio=limit,change_percent=100*(c/b-1))
 k=metrics['on_native_kimera'];b=metrics['baseline_on_native_kimera']
 ratio('Kimera_ATE_20pct_gain',k['common_ATE_m'],b['common_ATE_m'],.8)
 ratio('Kimera_20s_rotation_20pct_gain',k['horizons']['20']['rotation_RMS_deg'],b['horizons']['20']['rotation_RMS_deg'],.8)
 for h in ('10','20'):ratio('Kimera_'+h+'s_gravity_axis_nonregression',k['horizons'][h]['gravity_axis_rotation_RMS_deg'],b['horizons'][h]['gravity_axis_rotation_RMS_deg'],1)
 for shift in ('0','1','5'):
  for field in ('heading_RMS_deg','terminal_heading_deg'):ratio('Kimera_anchor'+shift+'_'+field,abs(k['anchors'][shift][field]),abs(b['anchors'][shift][field]),1)
 for role,limit in (('native_glim',1.1),('fused_online',1.05)):
  c=metrics['on_'+role];o=metrics['baseline_on_'+role]
  ratio(role+'_ATE_nonregression',c['common_ATE_m'],o['common_ATE_m'],limit)
  for h in ('10','20'):ratio(role+'_'+h+'s_translation_nonregression',c['horizons'][h]['translation_RMS_m'],o['horizons'][h]['translation_RMS_m'],1.1)
 checks['coverage']=dict(passed=bool(secs>=.99*fullsupport),candidate_supported_s=secs,baseline_supported_s=fullsupport,minimum_ratio=.99)
 checks['operational_gates']=dict(passed=True,evidence='results.json plus original sensor/LOCAL/lifecycle/receiver/projected-factor gates')
 timing=record['cases']['on']
 for label,value,budget in [('GLIM_scan_p95_ms',timing['glim_timing']['after5s']['scan_ms']['p95'],500.),('Kimera_backend_p95_ms',timing['kimera_timing_after5s']['backend_ms']['p95'],1000.)]:
  checks[label]=dict(passed=bool(value<=budget),observed=value,budget_ms=budget)
 data=dict(passed=True,scientific_screen_passed=all(v['passed'] for v in checks.values()),checks=checks,metrics=metrics,
  coverage=dict(capture_s=428.75,common_supported_s=secs,baseline_supported_s=fullsupport,queries=len(ids),excluded=int((~valid).sum()),first_s=float((grid[ids[0]]-a.START)*1e-9),last_s=float((grid[ids[-1]]-a.START)*1e-9)),
  input_sha256=a.HASHES,reference='IE-TC shares CPT7; not independent prism',projection='fixed receiving ablation, not degeneracy detection',default_promoted=False)
 assert all(a.g.sha(Path(p))==h for p,h in a.HASHES.items())
 a.save(H/'decision.json',data);np.savez_compressed(H/'decision_series.npz',**arrays)
 lines=['CP-V9 FULL LEICA-2 VELODYNE DECISION REPORT','=========================================','',
  'One fresh projected CBS-ON capture versus saved CP-V4 OFF/original ON; no fresh OFF or repeat.',
  'Capture428.75s at0.20x; native GLIM/Kimera and concurrent ONLINE fusion. Delayed is separate.',
  'Fixed gravity-axis+translation 4D factor, dense M C M^T; no detector, reference-based gate or multiplier.',
  'New factor uses exact local Jacobian; original default approximate BetweenFactor unchanged.',
  'Unchanged mean-only covariance alignment; shared CPT7 IE-TC, not independent ground truth.',
  '', 'COMMON SUPPORTED GRID: '+json.dumps(data['coverage']), '',
  'Common metrics: ATE m | 1s tRPE m/rRPE deg | 10s t/r | 20s t/r | supported path m']
 for role in roles:
  for m in modes:
   d=metrics[m+'_'+role];h=d['horizons']
   lines.append(f"{m:12} {role:15} {d['common_ATE_m']:.6f} | "+' | '.join(f"{h[t]['translation_RMS_m']:.6f}/{h[t]['rotation_RMS_deg']:.6f}" for t in ('1','10','20'))+f" | {d['supported_path_m']:.3f}")
 lines+=['','PREDECLARED SCREEN: '+('PASS' if data['scientific_screen_passed'] else 'FAIL / do not promote')]
 for name,c in checks.items():lines.append(name+': '+json.dumps(c))
 lines+=['','Full-saved/native/ONLINE/DELAYED, all covariance/revision/timing evidence: RESULTS_REPORT.txt/results.json.',
  'A screen pass is not independent confirmation or a production policy. A failure rejects this single intervention; no sweep/retry.',
  'Stopped at this experiment. Baseline artifacts unchanged.']
 with (H/'DECISION_REPORT.txt').open('x') as f:f.write('\n'.join(lines)+'\n')
 print(json.dumps(dict(scientific_screen_passed=data['scientific_screen_passed'],coverage=data['coverage'],checks=checks),indent=2))
if __name__=='__main__':main()
