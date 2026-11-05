"""Read-only post-capture startup/physical-interval comparison; no optimizer."""
import csv, hashlib, importlib.util, io, json, math, sys, types
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.append('/home/yeranis/.local/share/pipx/venvs/evo/lib/python3.12/site-packages')
import numpy as np
from PIL import Image
from rosbags.highlevel import AnyReader
H=Path(__file__).resolve().parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
P=W/'runs/gnssfgo_spx3_stationary_preroll_cpk27_20261023/prepared'
OLD=W/'runs/gnssfgo_spx3_awake_full_cpk26_20261021/prepared'
EPOCH=1730565870872888651
D=W/'src/datasets/GrandTour/SPX-3'
GT=D/'2024-11-02-17-43-10_cpt7_ie_tc.bag'
HASHES={}
def pin(p):
    with Path(p).open('rb') as f:HASHES[str(p)]=hashlib.file_digest(f,'sha256').hexdigest()
def load_module(name,path):
    pin(path);s=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(s);s.loader.exec_module(m);return m
geo=load_module('geometry',H.parent/'fixed_lag_eig2_g2k_reference_fl4y_20261017/analyze.py')
def dump(path,data):
    with path.open('x') as f:json.dump(data,f,indent=2,allow_nan=False)
def stats(a):return geo.stats(a)
def first_native(root,ref):
    f=root/'kimera_odometry.csv';pin(f)
    r=next(csv.DictReader(f.open()));stamp=int(r['field.header.stamp'])
    return dict(stamp_ns=stamp,initial_velocity=[float(r['field.twist.twist.linear.'+k]) for k in 'xyz'],
                reference_speed_mps={str(dt):float(np.linalg.norm(ref.at(stamp+dt)[:3,3]-ref.at(stamp-dt)[:3,3])/(2*dt*1e-9)) for dt in (20_000_000,50_000_000)})
def raw_startup():
    changes=[];prev=None
    camera=D/'2024-11-02-17-43-10_alphasense.bag'
    with AnyReader([camera]) as reader:
        conns=[c for c in reader.connections if c.topic=='/boxi/alphasense/front_left/image_raw/compressed']
        for c,t,raw in reader.messages(connections=conns,start=EPOCH+25*10**9,stop=EPOCH+33*10**9):
            m=reader.deserialize(raw,c.msgtype)
            a=np.asarray(Image.open(io.BytesIO(bytes(m.data))).convert('L').resize((320,240)),dtype=float)
            if prev is not None:changes.append([(t-EPOCH)*1e-9,float(np.mean(np.abs(a-prev)))])
            prev=a
    imu=[]
    with AnyReader([D/'2024-11-02-17-43-10_cpt7_imu.bag']) as reader:
        conns=[c for c in reader.connections if c.topic=='/boxi/cpt7/imu']
        for c,t,raw in reader.messages(connections=conns,start=EPOCH+25*10**9,stop=EPOCH+33*10**9):
            m=reader.deserialize(raw,c.msgtype);w=m.angular_velocity;a=m.linear_acceleration
            imu.append([(t-EPOCH)*1e-9,w.x,w.y,w.z,a.x,a.y,a.z])
    x=np.array(changes);y=np.array(imu);result={}
    for sec in range(25,33):
        a=x[(x[:,0]>=sec)&(x[:,0]<sec+1)];b=y[(y[:,0]>=sec)&(y[:,0]<sec+1)]
        result[str(sec)]=dict(image_mean_abs_adjacent_difference_8bit=stats(a[:,1]),
            gyro_norm_rad_s=stats(np.linalg.norm(b[:,1:4],axis=1)),accel_std_xyz_m_s2=np.std(b[:,4:],axis=0).tolist())
    return result
def beliefs(root,mode,ref):
    source=root/f'full_{mode}_source';rec=root/f'full_{mode}_receiver'
    times={int(r['field.header.seq']):int(r['field.header.stamp']) for r in csv.DictReader((source/'kimera_odometry.csv').open())}
    origin=int(next(csv.DictReader((rec/'receiver/online_states.csv').open()))['timestamp_ns'])
    groups={};payloads=rec/'events.csv';receipts=rec/'receipts.jsonl';pin(payloads);pin(receipts)
    with payloads.open() as f,receipts.open() as g:
        for raw,meta in zip(f,g):
            a=raw.strip().split(',')
            if a[0]!='B' or a[2]!='kimera_front':continue
            ti,tj=map(int,a[8:10])
            if not EPOCH+32*10**9<=ti<tj<=EPOCH+52*10**9:continue
            md=json.loads(meta);update=int(md['metadata']['source_update']);i,j=map(int,a[6:8])
            assert abs(ti-times[i])<1000 and abs(tj-times[j])<1000
            age=(times[update]-tj)*1e-9
            if age>4.80001:continue
            C=np.array(a[16:],float).reshape(6,6);mu=np.array(a[10:16],float)
            assert np.isfinite(C).all() and np.max(abs(C-C.T))<1e-12 and np.linalg.eigvalsh(C).min()>0
            Z=geo.exp(mu);target=geo.inv(ref.at(ti))@ref.at(tj);e=geo.log(geo.inv(Z)@target)
            r=dict(from_id=i,to_id=j,revision=int(a[5]),update=update,ti=ti,tj=tj,age_s=age,
                receiver_eligible=ti>=origin,translation_error_mm=float(np.linalg.norm(e[3:])*1000),
                rotation_error_deg=float(np.linalg.norm(e[:3])*180/np.pi),
                translation_rss_sigma_mm=float(np.sqrt(np.trace(C[3:,3:]))*1000),
                rotation_rss_sigma_deg=float(np.sqrt(np.trace(C[:3,:3]))*180/np.pi),
                reference_whitened_norm=float(np.sqrt(e@np.linalg.solve(C,e))),
                mean=mu.tolist(),covariance=C.tolist(),error=e.tolist())
            # Canonicalize with original integer native stamps, not float round trips.
            groups.setdefault((times[i],times[j]),[]).append(r)
    return groups
def cohort(rows):
    return {k:stats([r[k] for r in rows]) for k in ('translation_error_mm','rotation_error_deg',
        'translation_rss_sigma_mm','rotation_rss_sigma_deg','reference_whitened_norm')}
def trajectories(ref):
    # Reuse frozen evaluator arithmetic on the host. rosbag is only used by its
    # bag-loader, which is intentionally unavailable here; reference comes from
    # the validated rosbags reader above. Fail if the legacy loader is invoked.
    stub=types.ModuleType('rosbag')
    def forbidden(*a,**k):raise RuntimeError('Use the explicit rosbags reference')
    stub.Bag=forbidden;sys.modules['rosbag']=stub
    sys.path.insert(0,str(W/'src/cbsms/tools'))
    ev=load_module('frozen_evaluator',P/'evaluate_fl4f.py');rel=ev.rel
    pin(W/'src/cbsms/tools/score_grandtour_relative_pose.py');pin(W/'src/cbsms/tools/score_grandtour_prism.py')
    from scipy.spatial.transform import Rotation
    reference=(ref.times.astype(float)*1e-9,np.array([T[:3,3] for T in ref.poses]),
               Rotation.from_matrix(np.array([T[:3,:3] for T in ref.poses])).as_quat())
    curves={};saved={}
    for label,root in [('cold',OLD),('preroll',P)]:
        for mode in ('off','on'):
            for role,relative in [('glim',f'full_{mode}_source/odom_imu.txt'),
                    ('kimera',f'full_{mode}_source/kimera_odometry.csv'),
                    ('fusion_online',f'full_{mode}_receiver/receiver/online_trajectory.tum')]:
                p=root/relative;pin(p)
                loader=rel.load_odometry_csv if role=='kimera' else rel.load_tum
                c=rel.sort_unique(*loader(p));curves[f'{label}_{mode}_{role}']=c
    start=max(EPOCH*1e-9+32,max(c[0][0] for c in curves.values()))
    end=min(EPOCH*1e-9+57,min(c[0][-1] for c in curves.values()))
    grid=np.arange(math.ceil(start*10)/10,math.floor(end*10)/10+1e-8,.1)
    times=np.array([t for t in grid if all(rel.interpolate_pose(*c,t,.5) is not None for c in curves.values()) and rel.interpolate_pose(*reference,t,.05) is not None])
    assert len(times)>200
    for name,c in curves.items():saved[name]=ev.score(c,reference,times)
    # Host port of reference reading must reproduce the already completed
    # Noetic evaluator's full-saved numbers before comparative use.
    check=json.loads((P/'evaluation_full/summary.json').read_text())['results'];errors={}
    mapping={'glim':'native_glim','kimera':'native_kimera','fusion_online':'fused_online'}
    for name,c in curves.items():
        if not name.startswith('preroll_'):continue
        _,mode,role=name.split('_',2);r=ev.score_saved(c,reference)
        want=check[mode+'_'+mapping[role]]['full_saved']
        errors[name]=max(abs(r[k]-want[k]) for k in ('ATE_RMSE_m','translation_RPE_1s_RMSE_m','rotation_RPE_1s_RMSE_deg'))
        assert errors[name]<1e-6,(name,errors[name])
    segments=np.split(times,np.flatnonzero(np.diff(times)>.10001)+1)
    return dict(results=saved,rows=len(times),start_ns=int(times[0]*1e9),end_ns=int(times[-1]*1e9),
        envelope_s=float(times[-1]-times[0]),supported_contiguous_s=float(sum(s[-1]-s[0] for s in segments)),
        excluded_queries=len(grid)-len(times),same_evaluator_crosscheck_max=errors,
        includes_retrospective_fusion=False)
def main():
    assert json.loads((H/'status.json').read_text())['state']=='complete'
    pin(H/'PROTOCOL.txt');pin(Path(__file__));pin(GT)
    assert geo.self_test()
    ref=geo.Reference(GT)
    init={}
    for label,root in [('cold',OLD),('preroll',P)]:
        for mode in ('off','on'):init[label+'_'+mode]=first_native(root/f'full_{mode}_source',ref)
    for label,stem,bag in [('ETH1','gnssfgo_eth1_awake_full_cpk23_20261021',W/'src/datasets/GrandTour/2024-10-01-11-29-55_cpt7_ie_tc.bag'),
            ('EIG-2','gnssfgo_eig2_full_cpk24_20261021',W/'src/datasets/GrandTour/EIG-2/2024-11-03-13-59-54_cpt7_ie_tc.bag')]:
        pin(bag);other=geo.Reference(bag)
        init[label]=first_native(W/'runs'/stem/'prepared/full_off_source',other)
    allgroups={(label,mode):beliefs(root,mode,ref) for label,root in [('cold',OLD),('preroll',P)] for mode in ('off','on')}
    shared=set.intersection(*(set(g) for g in allgroups.values()))
    assert shared
    paired=[];summaries={}
    for stage,age in [('first',0.),('near4s',4.),('last4p8',4.8)]:
        selected={}
        for key in sorted(shared):
            choice={k:min(g[key],key=lambda r:abs(r['age_s']-age)) for k,g in allgroups.items()}
            if any(abs(r['age_s']-age)>.10001 for r in choice.values()):continue
            if max(r['age_s'] for r in choice.values())-min(r['age_s'] for r in choice.values())>1e-5:continue
            selected[key]=choice
        assert selected
        summaries[stage]=dict(intervals=len(selected),
            groups={f'{label}_{mode}':cohort([v[label,mode] for v in selected.values()]) for label,mode in allgroups})
        for key,choice in selected.items():
            paired.append(dict(stage=stage,native_start_ns=key[0],native_end_ns=key[1],
                groups={f'{a}_{b}':r for (a,b),r in choice.items()}))
    selected_startup=[p for p in paired if p['groups']['cold_off']['from_id'] in (0,1,2)]
    output=dict(initialization=init,raw_sensor_startup=raw_startup(),
        interval_counts={f'{a}_{b}':len(g) for (a,b),g in allgroups.items()},shared_intervals=len(shared),
        relative_summary=summaries,previous_three_edges=selected_startup,paired_versions=paired,
        trajectories=trajectories(ref),reference_bracket_max_ns=ref.max_bracket_ns,
        caveats=['IE-TC shares CPT7; no independent calibration claim',
                 'Reference-normalized errors are not optimizer residuals or NIS',
                 'Pre-roll changes both source histories and CBS, not solely the velocity prior',
                 'Cold final poses omitted: they have later future information'],input_sha256=HASHES)
    for p,want in HASHES.items():
        with Path(p).open('rb') as f:assert hashlib.file_digest(f,'sha256').hexdigest()==want,p
    dump(H/'startup_comparison.json',output)
    print(json.dumps({k:v for k,v in output.items() if k not in ('paired_versions','input_sha256','raw_sensor_startup')},indent=2))
if __name__=='__main__':main()
