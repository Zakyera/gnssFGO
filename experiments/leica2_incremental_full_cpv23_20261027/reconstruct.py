"""Causal publication-only products from exact-parity saved graph snapshots."""
import csv,hashlib,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

H=Path(__file__).resolve().parent
R=Path('/home/yeranis/results/gnssfgo_leica2_incremental_full_cpv23_20261027')
TP=np.array([.308837802,-.038312997,-.234944116])
pins={};out={'cases':{},'input_sha256':pins};arrays={}
def pin(p):
    with p.open('rb') as f:pins[str(p)]=hashlib.file_digest(f,'sha256').hexdigest()
    return p
def inv(T):
    a=np.eye(4);a[:3,:3]=T[:3,:3].T;a[:3,3]=-a[:3,:3]@T[:3,3];return a
def point(T):return T[:3,3]+T[:3,:3]@TP
def pose(row):
    a=np.eye(4);a[:3,3]=[float(row[k]) for k in ('px','py','pz')]
    a[:3,:3]=Rotation.from_quat([float(row[k]) for k in ('qx','qy','qz','qw')]).as_matrix();return a

assert json.loads(pin(H/'replay_complete.json').read_text())['pass_']
for mode in ('off','on'):
    assert json.loads(pin(H/f'{mode}_parity.json').read_text())['pass_']
    with pin(R/f'{mode}_publications.csv').open() as f:rows=list(csv.DictReader(f))
    snapshots={}
    for row in rows:snapshots.setdefault(int(row['newest']),[]).append(row)
    assert sorted(snapshots)==list(range(4282))
    G=[];O=[];D=[];C=[];timestamps=[];arrival=[];edges=[];revised=[]
    for k,snapshot in snapshots.items():
        assert [int(r['index']) for r in snapshot]==([0] if k==0 else [k-1,k])
        current=pose(snapshot[-1]);G.append(current)
        ts=int(snapshot[-1]['physical_ns']);av=int(snapshot[-1]['availability_ns'])
        assert av>=ts
        timestamps.append(ts);arrival.append(av)
        if k==0:O.append(current.copy());C.append(current.copy())
        else:
            prev=pose(snapshot[0]);assert snapshot[0]['event']==snapshot[1]['event']
            O.append(O[-1]@inv(prev)@current)
            C.append(C[-1]@inv(G[k-1])@current)
            edges.append(float(np.linalg.norm(point(current)-point(prev))))
            revised.append(point(prev)-point(G[k-1]))
        D.append(current@inv(O[-1]))
    G,O,D,C=map(np.asarray,(G,O,D,C));t=np.asarray(timestamps,np.int64);av=np.asarray(arrival,np.int64)
    assert np.all(np.diff(t)==100000000) and np.all(np.diff(av)>=0)
    baseline=np.loadtxt(pin(R/mode/'online_trajectory.tum'))
    assert len(baseline)==len(t)
    baseline_pose=np.tile(np.eye(4),(len(t),1,1));baseline_pose[:,:3,3]=baseline[:,1:4]
    baseline_pose[:,:3,:3]=Rotation.from_quat(baseline[:,4:8]).as_matrix()
    pose_error=float(np.max(np.abs(G-baseline_pose)));assert pose_error<1e-12
    telescoping=float(np.max(np.abs(C-G)));assert telescoping<1e-8
    reconstruction=float(np.max(np.abs(D@O-G)));assert reconstruction<1e-8
    cp=np.array([point(a) for a in O]);gp=np.array([point(a) for a in G])
    edge_error=float(np.max(np.abs(np.linalg.norm(np.diff(cp,axis=0),axis=1)-edges)));assert edge_error<1e-9
    if mode=='off':
        previous=np.load(pin(H.parent/'leica2_paired_receiver_trace_cpv22a_20261027/incremental_publication_states.npz'))
        prior_error=float(np.max(np.abs(O[:596]-previous['velodyne_chained_body_T'])))
        assert prior_error<1e-9
    else:prior_error=None
    dest=R/mode/'chained_trajectory.tum'
    with dest.open('x') as f:
        quat=Rotation.from_matrix(O[:,:3,:3]).as_quat()
        for i,stamp in enumerate(t):
            fields=[*O[i,:3,3],*quat[i]]
            f.write(f'{stamp//10**9}.{stamp%10**9:09d} '+' '.join(format(x,'.17g') for x in fields)+'\n')
    for key,value in [('timestamp_ns',t),('availability_ns',av),('graph_body_T',G),('chained_body_T',O),('graph_T_odom',D),('same_snapshot_prism_edge_m',np.asarray(edges)),('previous_state_correction',np.asarray(revised))]:
        arrays[f'{mode}_{key}']=value
    out['cases'][mode]={'rows':len(t),'body_pose_parity_max':pose_error,'published_control_max':telescoping,
        'graph_T_odom_reconstruction_max':reconstruction,'same_snapshot_edge_length_max_m':edge_error,
        'CPV22A_first_minute_pose_max':prior_error,'chained_path_m':float(np.linalg.norm(np.diff(cp,axis=0),axis=1).sum()),
        'absolute_path_m':float(np.linalg.norm(np.diff(gp,axis=0),axis=1).sum()),
        'final_chained_graph_position_difference_m':float(np.linalg.norm(cp[-1]-gp[-1])),
        'final_chained_graph_orientation_difference_deg':float(np.rad2deg(Rotation.from_matrix(O[-1,:3,:3].T@G[-1,:3,:3]).magnitude())),
        'chained_trajectory':str(dest),'chained_sha256':hashlib.sha256(dest.read_bytes()).hexdigest()}
pin(Path(__file__))
for p,s in pins.items():
    with open(p,'rb') as f:assert hashlib.file_digest(f,'sha256').hexdigest()==s
np.savez_compressed(H/'publication_products.npz',**arrays)
out['publication_products_sha256']=hashlib.sha256((H/'publication_products.npz').read_bytes()).hexdigest()
out['semantics']='Distinct causal odometry output; no future observations. Graph_T_odom times chained body pose equals original graph body pose. No chained covariance/velocity assigned.'
with (H/'reconstruction_checks.json').open('x') as f:json.dump(out,f,indent=2)
print(json.dumps(out['cases'],indent=2))
