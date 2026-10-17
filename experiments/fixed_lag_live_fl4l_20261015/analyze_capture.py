"""Read the single private snapshot; no estimator or receiver is run."""
import csv,json,math,re,sys,hashlib
from pathlib import Path
from collections import defaultdict
import numpy as np

W=Path('/workspace/cbs_gtsam4.3')
root=W/'runs/fixed_lag_same_graph_fl4l_20261015'
capture=root/'capture/update_438'
source=root/'on_source_capture'
sys.path.insert(0,str(W/'src/cbsms/tools'))
import score_grandtour_relative_pose as rel

def pose(v):
    return np.array(v[:3],float),rel.quaternion_to_matrix(np.array(v[4:7]+[v[3]],float))
def between(a,b): return a[1].T@(b[0]-a[0]),a[1].T@b[1]
def error(a,b):
    return dict(translation_m=float(np.linalg.norm(a[0]-b[0])),
        rotation_deg=math.degrees(math.acos(float(np.clip((np.trace(a[1].T@b[1])-1)/2,-1,1)))))
def exp(x):
    x=np.asarray(x);w=x[:3];rho=x[3:];angle=np.linalg.norm(w)
    wx,wy,wz=w;M=np.array([[0,-wz,wy],[wz,0,-wx],[-wy,wx,0.]])
    if angle<1e-8: R=np.eye(3)+M+.5*M@M;V=np.eye(3)+.5*M+M@M/6
    else:
        R=np.eye(3)+math.sin(angle)/angle*M+(1-math.cos(angle))/angle**2*M@M
        V=np.eye(3)+(1-math.cos(angle))/angle**2*M+(angle-math.sin(angle))/angle**3*M@M
    return V@rho,R
def read_values(path):
    out={}
    for r in csv.DictReader(path.open()):
        if r['type']=='pose':out[r['key']]=pose([r[k] for k in ['p0','p1','p2','qw','qx','qy','qz']])
    return out
assert (capture/'status.txt').read_text().splitlines()[-1]=='complete'
alignment=[]
for line in (source/'estimators.log').open():
    if 'KIMERA_CBS_MEAN_ALIGNMENT_ROW,438,' in line:
        v=line.split('KIMERA_CBS_MEAN_ALIGNMENT_ROW,')[1].strip().split(',')
        alignment.append(v)
assert len(alignment)==1,len(alignment)
v=alignment[0];i,j=map(int,v[3:5]);times=np.array(list(map(int,v[5:10])),dtype=np.int64)*1e-9
native=rel.sort_unique(*rel.load_reference_bag(W/'src/datasets/GrandTour/2024-10-01-11-29-55_cpt7_ie_tc.bag','/boxi/inertial_explorer/tc/odometry')[:3])
truth=between(rel.interpolate_pose(*native,times[2],.05),rel.interpolate_pose(*native,times[3],.05))
aligned=exp([float(x) for x in re.split('[:;]',v[11])])
objective={k:float(val) for k,val in (s.split('=',1) for s in (capture/'objective.txt').read_text().splitlines())}
states={};families={}
for name in ['before','production','reference']:
    values=read_values(capture/('values_'+name+'.csv'))
    z=between(values['x'+str(i)],values['x'+str(j)])
    states[name]={'target_error_vs_IE_TC':error(z,truth),'target_disagreement_vs_GLIM':error(z,aligned)}
    groups=defaultdict(lambda:dict(count=0,cost=0.,visual_valid=0))
    for line in (capture/(name+'_factors.txt')).read_text().splitlines():
        a=line.split();g=groups[a[1]];g['count']+=1;g['cost']+=float(a[2]);g['visual_valid']+=int('visual_valid=1' in a)
    families[name]=dict(groups)
    assert abs(sum(g['cost'] for g in groups.values())-objective[name+'_cost'])<1e-6*(1+objective[name+'_cost'])
original=json.loads((W/'runs/gnssfgo_eth1_fixed_lag_live120_fl4g_20261015/audit_fl4h/factor_details_v2.json').read_text())
historical=next(x for x in original['examples'] if x['update']==438)
report={'update':438,'receiver_edge':[i,j],'timestamps':times.tolist(),
        'aligned_G_error_vs_IE_TC':error(aligned,truth),'objective':objective,
        'states':states,'families':families,
        'historical_target_error_before_m':historical['prior_injected_edge_error_m'],
        'historical_target_error_after_m':historical['after_injected_edge_error_m'],
        'limits':['IE-TC shares CPT7; no independent-GT or trajectory-ATE claim',
                  'Fresh coupled replay is not necessarily bitwise historical reproduction',
                  'LM has at most30 iterations; residual gradient and cap must be reported',
                  'Only a cloned same-factor original-chart objective, not a proposed production change']}
with (root/'analysis.json').open('x') as out:json.dump(report,out,indent=2,allow_nan=False)
print(json.dumps(report,indent=2))
