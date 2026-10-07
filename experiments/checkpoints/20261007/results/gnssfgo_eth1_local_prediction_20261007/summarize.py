from pathlib import Path
import csv,json,hashlib,subprocess
import numpy as np
ROOT=Path(__file__).resolve().parent;OLD=ROOT.parent/'gnssfgo_eth1_roundoff_40s_20261006';LONG=ROOT.parent/'gnssfgo_eth1_wnoa_wnoj_120s_20261006'
def rows(p):
 with p.open()as f:return list(csv.DictReader(f))
def sha(p):
 h=hashlib.sha256()
 with p.open('rb')as f:
  for b in iter(lambda:f.read(2**20),b''):h.update(b)
 return h.hexdigest()
out={}
out['cases']={}
for model in ['wnoa','wnoj']:
 for mode in ['off','on']:
  label=mode+'_'+model;case=ROOT/label;v=ROOT/f'validation_{label}.json'
  if not v.exists():continue
  valid=json.loads(v.read_text());cmd=json.loads((ROOT/f'command_{label}.json').read_text())
  out['cases'][label]=dict(pass_all=valid['pass_all'],complete=valid['complete'],publications=valid['publications'],max_gradient=valid['max_gradient'],stable_steps=len(valid['stable_steps']),checkpoints=len(valid['checkpoints']),failures=valid['failures'],seconds=cmd['seconds'],maximum_rss_kib=cmd['maximum_rss_kib'])
  out['cases'][label]['recovery_steps']=[r for r in rows(case/'publication_steps.csv')if r['recovery']=='1']
  out['cases'][label]['consensus_steps']=valid.get('consensus_steps',[])
  out['cases'][label]['prefix']=valid['prefix']
  out['cases'][label]['numerical_prefix']=valid.get('numerical_prefix',{})
  out['cases'][label]['largest_checkpoint_differences']={k:max(c['difference'][k]for c in valid['checkpoints'])for k in valid['checkpoints'][0]['difference']}
  if not valid['complete']:continue
  from scipy.spatial.transform import Rotation
  states=rows(case/'online_states.csv');out['cases'][label]['online_path_m']=float(np.linalg.norm(np.diff(np.array([[float(r[k])for k in ['px','py','pz']]for r in states]),axis=0),axis=1).sum())
  out['cases'][label]['max_online_speed_mps']=float(np.linalg.norm(np.array([[float(r[k])for k in ['vx','vy','vz']]for r in states]),axis=1).max())
  transitions=[]
  for i in range(6,int(valid['summary']['support_states'])):
   a=rows(case/f'snapshot_{i-1}/incremental_states.csv');b=rows(case/f'snapshot_{i}/incremental_states.csv')
   for label2,index in [('current',i-1)]:
    p=np.array([float(b[index][k])-float(a[index][k])for k in ['px','py','pz']]);v=np.array([float(b[index][k])-float(a[index][k])for k in ['vx','vy','vz']]);transitions.append((np.linalg.norm(p),np.linalg.norm(v)))
  a=np.array(transitions)
  out['cases'][label]['same_time_next_publication_correction']={k:dict(median=float(np.median(a[:,j])),rms=float(np.sqrt(np.mean(a[:,j]**2))),p95=float(np.quantile(a[:,j],.95)),maximum=float(a[:,j].max()))for j,k in enumerate(['position_m','velocity_mps'])}
m=json.loads((ROOT/'manifest.json').read_text());bad=[p for p,h in m['protected'].items()if not Path(p).is_file()or sha(Path(p))!=h]
out['protected_files']=len(m['protected']);out['protected_mismatches']=bad
out['head']=subprocess.check_output(['git','-C','/home/yeranis/repos/gnssFGO','rev-parse','HEAD'],text=True).strip()
out['binaries']={model:sha(ROOT/f'build/{model}/replay')for model in ['wnoa','wnoj']}
(ROOT/'analysis.json').write_text(json.dumps(out,indent=2)+'\n')
print(json.dumps(out,indent=2))
assert not bad
