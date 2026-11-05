"""Supplement: actual source-to-source insertion matrices, distinct from revisions."""
import hashlib,json,re
from pathlib import Path
import numpy as np
H=Path(__file__).resolve().parent;E=H.parent
P=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/eth1_prior_fix_live_cpv103_20261103/prepared')
S=P/'full_on_source'; pins={}
def pin(p):
    p=Path(p)
    with p.open('rb') as f:pins[str(p)]=hashlib.file_digest(f,'sha256').hexdigest()
    return p
def read(p):return json.loads(pin(p).read_text())
def dec(line,tag):return re.sub(r'\x1b\[[0-9;]*m','',line.split(tag+',',1)[1]).strip().split(',')
def mat(s):return np.array([list(map(float,r.split(':'))) for r in s.split(';')])
def stats(x):return dict(n=len(x),min=float(np.min(x)),median=float(np.median(x)),mean=float(np.mean(x)),p95=float(np.percentile(x,95)),max=float(np.max(x)))
assert not (H/'insertions.json').exists()
cal=read(P/'transport_tools/frame_calibration.json');T=np.array(cal['T_alphasense_cpt7'])
R=T[:3,:3].T;t=-R@T[:3,3];x,y,z=t;sk=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
A=np.zeros((6,6));A[:3,:3]=A[3:,3:]=R;A[3:,:3]=sk@R
frame=read(E/'eth1_prior_fix_live_cpv103_20261103/frame_on.json');assert frame['passed']
coverage=read(S/'source_coverage.json');assert coverage['passed']
logs={d:next(p for p in (S/'ros_logs').glob(glob) if p.parent.name!='latest')
    for d,glob in [('G2K','*/kimera_vio_ros-*-stdout.log'),('K2G','*/glim_rosnode-1-stdout.log')]}
factors={};g2k_align={};frozen={};transport={};k2g_align={};injections=0
for line in pin(logs['G2K']).open():
    if 'CBS_ODOM_FACTOR_MATRIX_ROW,' not in line:continue
    f=dec(line,'CBS_ODOM_FACTOR_MATRIX_ROW');assert f[0]=='G2K' and f[3]=='persistent_odom_applied'
    key=(int(f[1][1:]),int(f[2][1:]));assert key not in factors;factors[key]=mat(f[4])
for line in pin(S/'estimators.log').open():
    if 'KIMERA_CBS_MEAN_ALIGNMENT_ROW,' in line:
        f=dec(line,'KIMERA_CBS_MEAN_ALIGNMENT_ROW');key=tuple(map(int,f[3:5]));assert key not in g2k_align;g2k_align[key]=mat(f[12])
    if 'FL4M_FROZEN_FACTOR_ROW,' in line:
        f=dec(line,'FL4M_FROZEN_FACTOR_ROW');key=tuple(map(int,f[3:5]));assert key not in frozen;frozen[key]=mat(f[6])
for line in pin(logs['K2G']).open():
    if 'GLIM_CBS_SOURCE_TRANSPORT_ROW,' in line:
        f=dec(line,'GLIM_CBS_SOURCE_TRANSPORT_ROW');key=tuple(map(int,f[1:3]));assert key not in transport
        c,d=mat(f[4]),mat(f[5]);assert np.array_equal(c,d);transport[key]=d
    if 'GLIM_CBS_MEAN_ALIGNMENT_ROW,' in line:
        f=dec(line,'GLIM_CBS_MEAN_ALIGNMENT_ROW');key=tuple(map(int,f[:2]));assert key not in k2g_align;k2g_align[key]=mat(f[11])
    if 'GLIM_CBS_ODOM_INJECT_ROW,' in line:
        f=dec(line,'GLIM_CBS_ODOM_INJECT_ROW');assert f[-1]=='injected_persistent';injections+=1
assert factors.keys()==g2k_align.keys()==frozen.keys() and len(factors)==1472
assert transport.keys()==k2g_align.keys() and len(transport)==injections==1472
assert all(np.array_equal(c,g2k_align[k]) and np.array_equal(c,frozen[k]) for k,c in factors.items())
assert all(np.array_equal(c,k2g_align[k]) for k,c in transport.items())
out={}
for direction,tab in [('G2K',factors),('K2G',transport)]:
    C=np.array(list(tab.values()));D=np.array([A@c@A.T for c in C]) if direction=='G2K' else C
    assert np.isfinite(C).all() and np.linalg.eigvalsh(C).min()>0
    out[direction]=dict(count=len(C),native_receiver_body='Alpha' if direction=='G2K' else 'CPT7',
        finite_SPD=True,receiver_covariance_matches_saved_alignment_and_transport_exactly=True,
        native_rotation_trace_rad2=stats(np.trace(C[:,:3,:3],axis1=1,axis2=2)),
        native_translation_trace_m2=stats(np.trace(C[:,3:,3:],axis1=1,axis2=2)),
        CPT7_translation_trace_m2=stats(np.trace(D[:,3:,3:],axis1=1,axis2=2)))
data=dict(passed=True,sources=out,input_sha256=pins,
    scope='Actual CP103 source-source persistent insertions; not mature revision inputs to fusion.',
    limitation='Numerical payload integrity only. Mean-only endpoint time alignment does not propagate bridge uncertainty; independence/calibration not established.')
with (H/'insertions.json').open('x') as f:json.dump(data,f,indent=2)
print(json.dumps(out,indent=2))
