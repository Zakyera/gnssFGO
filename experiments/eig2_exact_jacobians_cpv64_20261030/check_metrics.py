"""Independent scalar/quaternion cross-check of CP64 SciPy response metrics."""
import hashlib,json,math
from pathlib import Path
H=Path(__file__).resolve().parent
P=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs')/H.name/'results'
inputs={}
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def read(p):
    inputs[str(p)]=sha(p);return json.loads(p.read_text())
def norm(v):return math.sqrt(math.fsum(x*x for x in v))
def diff(a,b):return [x-y for x,y in zip(a,b)]
def angle(a,b):
    q=[x/norm(a[3:]) for x in a[3:]];r=[x/norm(b[3:]) for x in b[3:]]
    w=math.fsum(x*y for x,y in zip(q,r));u=q[1:];v=r[1:]
    cross=[u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]]
    xyz=[q[0]*v[i]-r[0]*u[i]-cross[i] for i in range(3)]
    return 2*math.atan2(norm(xyz),abs(w))*180/math.pi
a=read(H/'analysis.json');previous=read(P/'previous.json');initial=read(P/'initial.json')
checks=[]
for arm in 'ABC':
    for phase in ['ordinary','refresh']:
        label=arm+'_'+phase;result=read(P/(label+'_result.json'))
        refs={'old_state_revision':previous,'all_state_from_initial':initial,'actual_step':read(P/(label+'_start.json')),
              'versus_original':read(P/('A_'+phase+'_result.json'))}
        for name,base in refs.items():
            measured={}
            for k in base.keys()&result.keys():
                if k[0]=='x':measured[k]={'position_m':norm(diff(result[k][:3],base[k][:3])),'rotation_deg':angle(base[k],result[k])}
                elif k[0]=='v':measured[k]={'velocity_m_s':norm(diff(result[k],base[k]))}
                else:measured[k]={'accel_bias_m_s2':norm(diff(result[k][:3],base[k][:3])),'gyro_bias_rad_s':norm(diff(result[k][3:],base[k][3:]))}
                for m,v in measured[k].items():
                    expected=a['metrics'][label][name]['per_state'][k][m];err=abs(v-expected)/(1+abs(expected))
                    checks.append(dict(label=label,comparison=name,key=k,metric=m,error=err,passed=err<1e-8))
            for m,v in a['metrics'][label][name]['maximum'].items():
                actual=max(z[m] for z in measured.values() if m in z);err=abs(actual-v)/(1+abs(v))
                checks.append(dict(label=label,comparison=name,key='maximum',metric=m,error=err,passed=err<1e-8))
inputs[str(Path(__file__))]=sha(__file__)
out=dict(passed=all(x['passed'] for x in checks),count=len(checks),max_relative=max(x['error'] for x in checks),checks=checks,input_sha256=inputs)
with (H/'metrics_check.json').open('x') as f:json.dump(out,f,indent=2)
print(json.dumps({k:v for k,v in out.items() if k not in ['checks','input_sha256']},indent=2))

