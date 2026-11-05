"""Independent dense Schur and scalar/quaternion checks; not trajectory scoring."""
import csv,datetime,hashlib,json,math,subprocess,sys
from pathlib import Path
import numpy as np
H=Path(__file__).resolve().parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
K=W/'runs'/H.name
D=K/sys.argv[1]
assert json.loads((D/'passed.json').read_text())['passed']
def rel(a,b):return float(np.linalg.norm(a-b)/(1+np.linalg.norm(b)))
def eq(a,b):return max(rel(a[:-1,:-1],b[:-1,:-1]),rel(a[:-1,-1:],b[:-1,-1:]))
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def states(p):
    with p.open() as f:return {r[0]:list(map(float,r[1:])) for r in csv.reader(f)}
def norm(v):return math.sqrt(sum(x*x for x in v))
def quat_angle(a,b):
    a=[x/norm(a) for x in a];b=[x/norm(b) for x in b]
    # Relative quaternion vector; atan2 retains precision for tiny angles.
    w,x,y,z=a;v,i,j,k=b
    q=[w*v+x*i+y*j+z*k,w*i-x*v-y*k+z*j,w*j+x*k-y*v-z*i,w*k-x*j+y*i-z*v]
    return 2*math.atan2(norm(q[1:]),abs(q[0]))
checks=[]
for root in (D/'release_fixtures',D/'san_fixtures',D/'results'):
    for path in sorted(root.glob('*_pre.csv')):
        prefix=path.name[:-8]
        pre=np.loadtxt(path,delimiter=',');post=np.loadtxt(root/(prefix+'_post.csv'),delimiter=',')
        gone=[];keep=[];cursor=0
        with (root/(prefix+'_ordering.txt')).open() as f:
            for key,dim,marginal in csv.reader(f):
                dim=int(dim);(gone if int(marginal) else keep).extend(range(cursor,cursor+dim));cursor+=dim
        assert cursor==len(pre)-1
        rr=keep+[cursor]
        hgg=pre[np.ix_(gone,gone)];hgr=pre[np.ix_(gone,rr)]
        schur=pre[np.ix_(rr,rr)]-pre[np.ix_(rr,gone)]@np.linalg.solve(hgg,hgr)
        error=eq(schur,post)
        assert error<1e-8,(prefix,error)
        # Full and marginalized Gaussian solutions must agree on survivors.
        full=np.linalg.solve(pre[:-1,:-1],pre[:-1,-1]);reduced=np.linalg.solve(post[:-1,:-1],post[:-1,-1])
        solution_error=float(np.linalg.norm(full[keep]-reduced))
        assert solution_error<1e-5,(prefix,solution_error)
        checks.append(dict(root=root.name,arm=prefix,Schur_H_rhs_relative=error,
                           Schur_constant_absolute=float(abs(schur[-1,-1]-post[-1,-1])),
                           survivor_delta_norm=solution_error))
root=D/'results';metrics={r[0]:float(r[1]) for r in csv.reader((root/'metrics.csv').open())}
old=W/'runs/eig2_retirement_capture_cpv66_20261031/causal/results/update_250'
parity={}
for phase in ('ordinary','refresh'):
    got=states(root/f'B_{phase}.csv');expected=states(old/f'B_{phase}.csv');assert got.keys()==expected.keys()
    maximum_position=maximum_angle=maximum_other=0.
    for key,a in got.items():
        b=expected[key]
        if key.startswith('x'):
            maximum_position=max(maximum_position,norm([x-y for x,y in zip(a[:3],b[:3])]))
            maximum_angle=max(maximum_angle,quat_angle(a[3:],b[3:]))
        else:maximum_other=max(maximum_other,norm([x-y for x,y in zip(a,b)]))
    assert max(maximum_position,maximum_angle,maximum_other)<1e-5
    parity[phase]=dict(position_m=maximum_position,rotation_rad=maximum_angle,other_norm=maximum_other)
a=np.loadtxt(root/'A_local.csv',delimiter=',');b=np.loadtxt(root/'fixed_chart_retained_local.csv',delimiter=',')
delta=b-a;difference=rel(b,a)
assert abs(difference-metrics['real_LOCAL_same_chart_difference'])<1e-12
eigen=np.linalg.eigvalsh((delta[:-1,:-1]+delta[:-1,:-1].T)/2)
info=dict(relative_augmented_information_change=difference,
          information_change_frobenius=float(np.linalg.norm(delta[:-1,:-1])),
          rhs_change_norm=float(np.linalg.norm(delta[:-1,-1])),
          eigen_min=float(eigen[0]),eigen_max=float(eigen[-1]),
          live_gate_passed=bool(metrics['live_provenance_gate_passed']))
assert not info['live_gate_passed'], 'unexpected provenance result requires review, not automatic source launch'
pins=json.loads((D/'inputs.json').read_text())
for p,h in pins.items():assert sha(p)==h,p
containers=subprocess.check_output(['docker','inspect','-f','{{.Name}} {{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'],text=True)
assert containers=='/cbsms_ws_gpu exited\n/ubuntu-jammy-humble-nvidia exited\n'
for receipt in [D/'start.json',*D.glob('*_process.json')]:
    pid=json.loads(receipt.read_text())['pid'];assert not Path('/proc',str(pid)).exists(),(receipt,pid)
hesai=subprocess.check_output(['python3',str(H.parents[1]/'experiments/checkpoints/20261023_hesai_four_sequences/verify_checkpoint.py'),'--originals'],text=True)
result=dict(passed=True,finished=datetime.datetime.now().astimezone().isoformat(),
            native_metrics=metrics,independent_Schur_checks=checks,CP66_retained_branch_parity=parity,
            provenance=info,protected_input_files=len(pins),containers=containers,owned_jobs_exited=True,
            original_Hesai_verification=hesai,sources_launched=0,forward_continuation=False,
            source_covariance_tuning=False,production_changes=False)
with (H/'verification.json').open('x') as f:json.dump(result,f,indent=2)
print(json.dumps(result,indent=2))
