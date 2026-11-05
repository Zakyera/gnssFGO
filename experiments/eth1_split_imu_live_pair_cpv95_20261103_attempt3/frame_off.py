"""Saved-payload frame audit only. Never starts ROS or alters estimator data."""
import hashlib,importlib.util,json,sys,xml.etree.ElementTree as ET
from pathlib import Path
from collections import defaultdict
import numpy as np
from scipy.spatial.transform import Rotation
sys.dont_write_bytecode=True
sys.path.append('/home/yeranis/.local/share/pipx/venvs/rosbags/lib/python3.12/site-packages')
from rosbags.highlevel import AnyReader
H=Path(__file__).resolve().parent;E=H.parent;W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
P=W/'runs'/H.name/'prepared';pins={}
def pin(p):
    p=Path(p)
    with p.open('rb') as f:pins[str(p)]=hashlib.file_digest(f,'sha256').hexdigest()
    return p
s=importlib.util.spec_from_file_location('cp91_geometry',pin(E/'eig2_hesai_onset_cpv50_20261030/analyze.py'))
a=importlib.util.module_from_spec(s);s.loader.exec_module(a)
def decode(line,tag):return a.ANSI.sub('',line.split(tag+',',1)[1]).strip().split(',')
def adj(T):
    R=T[:3,:3];A=np.zeros((6,6));A[:3,:3]=R;A[3:,3:]=R;A[3:,:3]=a.skew(T[:3,3])@R;return A
def err(A,B):return float(np.max(abs(A-B)))
def stats(v):
    v=np.asarray(v,float);return dict(n=len(v),maximum=float(v.max()) if len(v) else None,median=float(np.median(v)) if len(v) else None,p95=float(np.percentile(v,95)) if len(v) else None)
def main():
    mode=sys.argv[1];assert mode=='off'
    dest=H/'frame_off.json';assert not dest.exists()
    S=P/'full_off_source'
    pin(Path(__file__))
    audit=json.loads(pin(P/'input_audit.json').read_text());auth=np.asarray(audit['T_alphasense_cpt7'])
    xml=ET.parse(pin(P/mode/'active_window_on.launch'))
    node=next(x for x in xml.iter('node') if x.attrib.get('name')=='grandtour_alpha_imu_to_cpt7_imu_tf')
    args=node.attrib['args'].split();assert args[-2:]==['alphasense_imu','cpt7_imu']
    X=np.eye(4);X[:3,3]=list(map(float,args[:3]));X[:3,:3]=Rotation.from_quat(np.array(args[3:7],float)).as_matrix()
    assert err(X,auth)<1e-9;Y=a.inv(X);AY=adj(Y)
    prior=json.loads(pin(E/'eig2_frame_extrinsic_cpv55_20261030/native_attempt1/native_test.json').read_text());assert prior['passed']
    for p,d in prior['files'].items():assert pins.get(str(pin(p)))==d,p
    rng=np.random.default_rng(91);jac=[];roundtrip=[];wrong=[]
    for i in range(30):
        M=a.exp(rng.normal(size=6)*.2);C0=rng.normal(size=(6,6));C=C0@C0.T+np.eye(6)*.1
        N=Y@M@X;D=AY@C@AY.T
        roundtrip += [err(X@N@Y,M),err(adj(X)@D@adj(X).T,C)]
        J=np.column_stack([a.log(a.inv(N)@(Y@(M@a.exp(np.eye(6)[j]*1e-7))@X))/1e-7 for j in range(6)])
        jac.append(err(J,AY));wrong.append(err(N,M))
    assert max(roundtrip)<1e-10 and max(jac)<1e-7 and min(wrong)>1e-3
    exports={'g':defaultdict(list),'k':defaultdict(list)};parity={'g':[],'k':[]};aligned={'g':[],'k':[]}
    glogs=[p for p in (S/'ros_logs').glob('*/glim_rosnode-1-stdout.log') if p.parent.name!='latest'];assert len(glogs)==1
    for line in pin(S/'estimators.log').open():
        if 'KIMERA_CBS_REVISION_EDGE_ROW,' in line:
            f=decode(line,'KIMERA_CBS_REVISION_EDGE_ROW');assert len(f)==64
            key=tuple(map(int,f[2:4]));M=a.exp(f[22:28]);C=np.array(f[28:],float).reshape(6,6)
            parity['k'].append(err(a.inv(a.pose(f[8:15]))@a.pose(f[15:22]),M))
            exports['k'][key].append((Y@M@X,AY@C@AY.T))
        if 'KIMERA_CBS_MEAN_ALIGNMENT_ROW,' in line:
            f=decode(line,'KIMERA_CBS_MEAN_ALIGNMENT_ROW');aligned['k'].append((tuple(map(int,f[1:3])),a.exp(f[10].split(';'))))
    for line in pin(glogs[0]).open():
        if 'GLIM_CBS_EXPORT_ENDPOINT_ROW,' in line:
            f=decode(line,'GLIM_CBS_EXPORT_ENDPOINT_ROW');key=tuple(map(int,f[3:5]));poses=[]
            for off in (7,14):
                T=np.eye(4);T[:3,3]=np.array(f[off:off+3],float);T[:3,:3]=Rotation.from_quat(np.array(f[off+3:off+7],float)).as_matrix();poses.append(T)
            M=a.exp(f[21].split(';'));parity['g'].append(err(a.inv(poses[0])@poses[1],M));exports['g'][key].append((M,None))
        if 'GLIM_CBS_MEAN_ALIGNMENT_ROW,' in line:
            f=decode(line,'GLIM_CBS_MEAN_ALIGNMENT_ROW');aligned['g'].append((tuple(map(int,f[:2])),a.exp(f[9].split(';'))))
    wire={'g':defaultdict(list),'k':defaultdict(list)};meanerrors={'g':[],'k':[]};coverrors=[];unmatched={'g':0,'k':0};covs={'g':[],'k':[]};headers={'g':set(),'k':set()}
    topics={'/kimera/cbs/odom_belief_in':'g','/kimera/cbs/odom_belief_out':'k'}
    with AnyReader([pin(S/'recordings/cbs.bag')]) as reader:
        for c,t,raw in reader.messages(connections=[c for c in reader.connections if c.topic in topics]):
            msg=reader.deserialize(raw,c.msgtype);agent=topics[c.topic]
            for z in msg.beliefs:
                assert int(z.source_agent)==ord(agent)
                key=(int(z.from_pose_index),int(z.to_pose_index));M=a.exp(np.array(z.relative_mu,copy=True));C=np.array(z.covariance,copy=True).reshape(6,6)
                wire[agent][key].append(M);covs[agent].append(C);headers[agent].add(z.header.frame_id)
                if not exports[agent][key]:unmatched[agent]+=1;continue
                candidates=exports[agent][key]
                if agent=='k':
                    # Both mean AND covariance must match the same native version.
                    diffs=[(err(M,N),err(C,D)/max(1e-30,float(np.max(abs(C))))) for N,D in candidates]
                    em,ec=min(diffs,key=lambda z:max(z[0]/1e-9,z[1]/1e-6));coverrors.append(ec)
                else:em=min(err(M,N) for N,_ in candidates)
                meanerrors[agent].append(em)
    receive={};covsum={}
    for receiver,sender in [('g','k'),('k','g')]:
        errors=[];miss=0
        for key,raw in aligned[receiver]:
            options=wire[sender][key]
            if not options:miss+=1;continue
            errors.append(min(err(raw,X@v@Y if receiver=='k' else v) for v in options))
        receive[receiver]=dict(error=stats(errors),unmatched=miss,logged_rows=len(aligned[receiver]))
        if mode=='on':assert errors and miss==0 and max(errors)<1e-9,receive[receiver]
    for agent,values in covs.items():
        C=np.array(values);assert len(C)>100 and np.isfinite(C).all()
        symmetry=float(np.max(abs(C-C.transpose(0,2,1))));ev=float(np.linalg.eigvalsh(C).min());assert symmetry<1e-10 and ev>0
        covsum[agent]=dict(rows=len(C),symmetry_max=symmetry,eigenvalue_min=ev,
            rotation_trace_rad2=stats(np.trace(C[:,:3,:3],axis1=1,axis2=2)),translation_trace_m2=stats(np.trace(C[:,3:,3:],axis1=1,axis2=2)))
        assert meanerrors[agent] and max(meanerrors[agent])<1e-9 and unmatched[agent]==0,(agent,stats(meanerrors[agent]),unmatched[agent])
        assert parity[agent] and max(parity[agent])<1e-9
    assert coverrors and max(coverrors)<1e-6
    data=dict(passed=True,mode=mode,source=str(S),T_alphasense_cpt7=X.tolist(),author_transform_error=err(X,auth),
        right_tangent_jacobian_error=stats(jac),roundtrip_error=stats(roundtrip),identity_negative_control=stats(wrong),
        native_endpoint_parity={k:stats(v) for k,v in parity.items()},wire_mean_parity={k:stats(v) for k,v in meanerrors.items()},
        Kimera_wire_covariance_relative_error=stats(coverrors),unmatched_wire=unmatched,receiver_raw_match=receive,
        transported_covariance=covsum,wire_world_header_frames={k:sorted(v) for k,v in headers.items()},input_sha256=pins,
        limitations=['Native endpoint to wire mean/covariance joins search retained versions, not an independent arrival-order audit.',
        'GLIM native covariance not separately logged in endpoint rows; its transported covariance is checked finite/SPD, not native parity.',
        'OFF has no insertion check; ON checks logged pre-time-alignment receive means, not unlogged native final factors.',
        'Body conversion is separate from unchanged mean-only timestamp covariance alignment approximation.',
        'Persistent peer history and uncertainty calibration remain unqualified; this is not an independence proof.'])
    with dest.open('x') as f:json.dump(data,f,indent=2,allow_nan=False)
    print(json.dumps({k:v for k,v in data.items() if k not in ('input_sha256','limitations')},indent=2))
if __name__=='__main__':main()
