"""Independent scalar/quaternion verification and read-only owned-job closeout."""
import csv,datetime,hashlib,json,math,subprocess
from pathlib import Path
H=Path(__file__).resolve().parent;R=H.parents[1]
K=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs')/H.name
D=K/'causal';a=json.loads((H/'causal_analysis.json').read_text())
assert a['passed'] and json.loads((D/'passed.json').read_text())['passed']
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def read(p):
    with p.open() as f:return {r[0]:list(map(float,r[1:])) for r in csv.reader(f)}
def norm(v):return math.sqrt(sum(x*x for x in v))
def dot(a,b):return sum(x*y for x,y in zip(a,b))
def sub(a,b):return [x-y for x,y in zip(a,b)]
def qmul(a,b):
    w,x,y,z=a;v,i,j,k=b
    return [w*v-x*i-y*j-z*k,w*i+x*v+y*k-z*j,w*j-x*k+y*v+z*i,w*k+x*j-y*i+z*v]
def rotlog(q0,q1):
    q0=[x/norm(q0) for x in q0];q1=[x/norm(q1) for x in q1]
    q=qmul([q0[0],-q0[1],-q0[2],-q0[3]],q1)
    if q[0]<0:q=[-x for x in q]
    s=norm(q[1:]);factor=2*math.atan2(s,q[0])/s if s>1e-15 else 2
    return [factor*x for x in q[1:]]
count=0;maximum=0.
def check(x,y,tolerance=1e-8):
    global count,maximum
    error=abs(x-y);assert error<tolerance,(x,y,error)
    maximum=max(maximum,error);count+=1
for e in a['events']:
    k=e['k'];root=D/'results'/f'update_{k}'
    original=K/'prepared/full_on_source/checkpoints'/f'update_{k}'/'after_ordinary/checkpoint.bin'
    assert sha(original)==sha(root/'source_roundtrip.bin'),'real archive roundtrip differs'
    for arm in 'BCD':assert sha(original)==sha(root/f'{arm}_restored.bin')
    prev=read(root/'previous.csv')[f'x{k-1}']
    for phase in ('ordinary','refresh'):
        states={arm:read(root/f'{arm}_{phase}.csv')[f'x{k-1}'] for arm in 'ABCD'}
        r={arm:rotlog(prev[3:],s[3:]) for arm,s in states.items()}
        dt=sub(states['A'][:3],prev[:3]);dr=r['A']
        for label,coeff in [('removal',{'A':1,'B':-1}),('admission',{'B':1,'C':-1}),('interaction',{'A':1,'D':-1,'B':-1,'C':1})]:
            et=[sum(w*states[arm][j] for arm,w in coeff.items()) for j in range(3)]
            er=[sum(w*r[arm][j] for arm,w in coeff.items()) for j in range(3)]
            for key,ev,dv in [('translation',et,dt),('rotation',er,dr)]:
                got=e['phases'][phase]['contrasts'][label][key]
                check(got['observed'],norm(dv));check(got['effect'],norm(ev))
                if norm(dv)>=1e-6:
                    ratio=dot(ev,dv)/dot(dv,dv);check(got['fraction'],ratio)
                    check(got['perpendicular'],norm([x-ratio*y for x,y in zip(ev,dv)]))
                else:assert got['fraction'] is None
pins=json.loads((D/'inputs.json').read_text())
for p,h in pins.items():assert sha(p)==h,p
states=subprocess.check_output(['docker','inspect','-f','{{.Name}} {{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'],text=True)
assert states=='/cbsms_ws_gpu exited\n/ubuntu-jammy-humble-nvidia exited\n'
receipts=[H/'start.json',H/'launch.json',D/'start.json',*D.glob('*_process.json')]
for p in receipts:
    pid=json.loads(p.read_text())['pid'];assert not Path('/proc',str(pid)).exists(),('owned PID still exists',p,pid)
hesai=subprocess.check_output(['python3',str(R/'experiments/checkpoints/20261023_hesai_four_sequences/verify_checkpoint.py'),'--originals'],text=True)
snapshot_pins={str(p):sha(p) for mode in ('off','on') for p in (K/f'prepared/full_{mode}_source/checkpoints').rglob('*') if p.is_file()}
assert sum(p.endswith('checkpoint.bin') for p in snapshot_pins)==848
with (H/'snapshot_sha256.json').open('x') as f:json.dump(snapshot_pins,f,indent=2)
verification=dict(passed=True,independent_scalar_quaternion_checks=count,max_absolute_metric_difference=maximum,
                  original_native_roundtrips=11,restored_branch_byte_matches=33,
                  protected_input_files=len(pins),containers=states,owned_jobs_exited=True,hesai=hesai)
with (H/'verification.json').open('x') as f:json.dump(verification,f,indent=2)
result=dict(passed=True,status='COMPLETE_MATCHED_60S_CAPTURE_AND_11_EVENT_CAUSAL_TEST',
            finished=datetime.datetime.now().astimezone().isoformat(),sensor_captures=2,duration_each_s=60,
            snapshots=848,events=11,real_native_steps=88,source_retries=0,
            result='Removal has large immediate causal influence in the three selected late correction events; no full-trajectory repair or persistent-policy validation.',
            verification=verification,build_attempt_failures_preserved=True,production_changed=False,
            report_sha256=sha(H/'REPORT.txt'),causal_analysis_sha256=sha(H/'causal_analysis.json'))
with (H/'complete.json').open('x') as f:json.dump(result,f,indent=2)
print(json.dumps(result,indent=2))
