"""Join actual saved PoseBetween factors to admissions; preserve legacy outputs."""
import csv,hashlib,importlib.util,json
from pathlib import Path
import numpy as np
H=Path(__file__).resolve().parent;K=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs')/H.name
Q=K.parent/'eig2_hesai_refresh_full_cpv49_20261030/prepared';pins={}
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
spec=importlib.util.spec_from_file_location('cp50',H.parent/'eig2_hesai_onset_cpv50_20261030/analyze.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
assert json.loads((H/'factors_v2_passed.json').read_text())['passed']
data={mode:m.parse_source(K/f'prepared/full_{mode}_source/estimators.log') for mode in ('off','on')}
old=m.parse_source(Q/'full_on_source/estimators.log');original=json.loads((H/'analysis.json').read_text())
allpriors={};rows=[];maximum=0.;native_means={};phases={};comparisons=[]
for mode in ('off','on'):
 d=K/'factor_inspection_v2'/mode
 for p in d.glob('prior_*.bin'):allpriors[str(p)]=sha(p)
 p=d/'factors.csv';pins[str(p)]=sha(p)
 for r in csv.DictReader(p.open()):
  k=int(r['update']);i=int(r['from'][1:]);j=int(r['to'][1:]);mean=np.array(r['mean_matrix'].split(';'),float).reshape(4,4)
  admitting=[(a,v) for a,v in data[mode]['frozen'].items() if a[1:]==(i,j) and a[0]<=k-(r['phase']=='before_ordinary')]
  assert len(admitting)<=1,(mode,k,i,j)
  kind='cbs' if admitting else 'native_pose_between'
  if admitting:
   a,v=admitting[0];error=float(np.max(abs(mean-v['mean'])));maximum=max(maximum,error);assert error<1e-12
  else:
   key=(i,j)
   if key in native_means:assert np.array_equal(native_means[key],mean)
   native_means[key]=mean
  numeric={key:float(r[key]) for key in ('initial_cost','estimate_cost','initial_rotation_deg','initial_translation_log_m','estimate_rotation_deg','estimate_translation_log_m')}
  rows.append(dict(mode=mode,k=k,phase=r['phase'],slot=int(r['slot']),edge=[i,j],kind=kind,**numeric))
  dest=phases.setdefault((mode,k,r['phase']),dict(cbs=0,native_pose_between=0,cbs_cost=0.,native_cost=0.))
  dest[kind]+=1;dest['cbs_cost' if kind=='cbs' else 'native_cost']+=numeric['estimate_cost']
 for k in range(6,26):
  for phase in ('before_ordinary','after_ordinary','after_refresh'):
   d=phases[mode,k,phase];assert d['native_pose_between']==1
   assert d['cbs']==(0 if mode=='off' else k-5-(phase=='before_ordinary'))
assert len(allpriors)==120 and len(set(allpriors.values()))==1,'initial prior function changed'
for key,v in data['on']['aligned'].items():
 if key[0]>25:continue
 o=old['aligned'].get(key);assert o is not None
 comparisons.append(dict(k=key[0],ids_same=v['ids']==o['ids'],times_same=v['times']==o['times'],
  raw_mean_change=m.diff(v['raw'],o['raw']),aligned_mean_change=m.diff(v['mean'],o['mean'])))
prefix={}
for mode in ('off','on'):
 od=old if mode=='on' else m.parse_source(Q/'full_off_source/estimators.log')
 current=data[mode]['windows'];before=od['windows'];differences=[]
 for k,window in current.items():
  assert set(window)==set(before[k]),(mode,k)
  for key,T in window.items():differences.append(dict(k=k,key=key,exact=np.array_equal(T,before[k][key]),**m.diff(T,before[k][key])))
 prefix[mode]=dict(recorded_active_pose_comparisons=len(differences),all_exact=all(r['exact'] for r in differences),
  first_changed_update=next((r['k'] for r in differences if not r['exact']),None),
  max_position_m=max(r['translation_m'] for r in differences),max_rotation_deg=max(r['rotation_deg'] for r in differences))
out=dict(passed=True,actual_cbs_measurements_match_frozen_max_matrix_element_error=maximum,
 no_cbs_in_off=True,exactly_one_cbs_factor_added_per_observed_on_update=True,
 no_removal_in_observed_interval=True,initial_prior_snapshots=120,initial_prior_functions_byte_identical=True,
 initial_prior_sha256=next(iter(allpriors.values())),native_pose_between_edges=[list(k) for k in native_means],
 legacy_family_label_correction="inspection/* same_graph files call every PoseBetween peer_pose_between, including native no-motion x2->x3 in both modes. The initial_prior family also includes native zero-velocity v3 alongside x0/v0/b0 initial priors. This joined audit separates actual CBS factors; the four prior functions are checked unchanged.",
 phases=[dict(mode=k[0],k=k[1],phase=k[2],**v) for k,v in phases.items()],
 factors=rows,CPV49_admission_comparison=comparisons,CPV49_active_window_parity=prefix,input_sha256={**pins,**allpriors,**m.PINS})
with (H/'factor_audit.json').open('x') as f:json.dump(out,f,indent=2,allow_nan=False)
print({k:v for k,v in out.items() if k not in ('phases','factors','CPV49_admission_comparison','input_sha256')})
for row in rows:
 if row['mode']=='on' and row['k'] in (6,25) and row['kind']=='cbs':print(row)
print('first-admission comparison',comparisons[0])
