"""Reuse sealed EVO error arrays; localize, do not re-score or optimize."""
import csv
import datetime
import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from evo.tools import file_interface

H=Path(__file__).resolve().parent
E=H.parent
S=E/'eth1_prior_fix_evaluation_cpv104_20261104'
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs')
ROOTS={'OFF':W/'eth1_split_imu_live_pair_cpv95_20261103_attempt3/prepared',
       'ON':W/'eth1_prior_fix_live_cpv103_20261103/prepared'}
PREFIX={'OFF':'CP95_OFF','ON':'CP103_ON'}
PINS={}
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def pin(p):
    p=Path(p);PINS[str(p)]=sha(p);return p
def read(p):return json.loads(pin(p).read_text())
def stats(x):
    x=np.asarray(x,float);assert x.size and np.isfinite(x).all()
    return dict(n=len(x),mean=float(x.mean()),median=float(np.median(x)),
                p95=float(np.percentile(x,95)),maximum=float(x.max()))
def save(name,v):
    with (H/name).open('x') as f:json.dump(v,f,indent=2,allow_nan=False)
def load_csv(p):
    with pin(p).open() as f:return list(csv.DictReader(f))

assert not (H/'localization.json').exists()
sealed=read(S/'complete.json')['output_sha256']
r=read(S/'results.json')
npz=np.load(pin(S/'evaluation/comparison.npz'))
valid=npz['common_valid'];ts=npz['timestamp_ns'][valid]
audit=read(ROOTS['ON']/'input_audit.json');epoch=audit['epoch_ns']
t=(ts-epoch)*1e-9
gp=npz['reference_position'][valid]
curves={};details={};vectors={};norms={}
for mode in ROOTS:
    for role in ('glim','kimera','fusion_online','fusion_delayed'):
        key=PREFIX[mode]+'_'+role
        row=r['common'][key];a=row['alignment']
        p=npz[key+'_position'][valid]@np.asarray(a['rotation']).T+np.asarray(a['translation'])
        error=p-gp
        z=file_interface.load_res_file(pin(S/'evaluation/common'/(key+'_ape.zip')))
        n=z.np_arrays['error_array']
        assert np.max(abs(np.linalg.norm(error,axis=1)-n))<1e-8
        assert abs(float(np.sqrt(np.mean(n*n)))-row['ATE_RMSE_m'])<1e-12
        total=float(np.sum(n*n));vectors[key]=error;norms[key]=n;curves[key]=p
        bins=[]
        for lo in range(0,330,30):
            use=(t>=lo)&(t<lo+30)
            if not use.any():continue
            bins.append(dict(start_s=lo,end_s=lo+30,n=int(use.sum()),
                RMS_existing_EVO_error_m=float(np.sqrt(np.mean(n[use]**2))),
                energy_fraction=float(np.sum(n[use]**2)/total),
                mean_error_xyz_m=error[use].mean(axis=0).tolist()))
        details[key]=dict(saved_ATE_RMSE_m=row['ATE_RMSE_m'],error=stats(n),
            axis_error_energy_fraction=(np.sum(error**2,axis=0)/total).tolist(),
            largest_error_relative_s=float(t[np.argmax(n)]),bins=bins)
delta={}
for role in ('fusion_online','fusion_delayed'):
    off=norms['CP95_OFF_'+role];on=norms['CP103_ON_'+role]
    dif=on**2-off**2;overall=float(dif.sum())
    bins=[]
    for lo in range(0,330,30):
        use=(t>=lo)&(t<lo+30)
        if use.any():bins.append(dict(start_s=lo,end_s=lo+30,
             extra_squared_error_sum=float(dif[use].sum()),
             fraction_net_extra_squared_error=float(dif[use].sum()/overall)))
    delta[role]=dict(net_extra_squared_error_sum=overall,bins=bins)

telemetry={}
for mode,P in ROOTS.items():
    base=P/('full_'+mode.lower()+'_receiver');R=base/'receiver'
    st=load_csv(R/'online_states.csv');fin=load_csv(R/'finalized_states.csv')
    sn=np.array([int(x['timestamp_ns']) for x in st],np.int64)
    assert np.array_equal(sn,np.array([int(x['timestamp_ns']) for x in fin],np.int64))
    p=np.array([[float(x[a]) for a in ('px','py','pz')] for x in st]);f=np.array([[float(x[a]) for a in ('px','py','pz')] for x in fin])
    rotations=Rotation.from_quat([[float(x[a]) for a in ('qx','qy','qz','qw')] for x in st])
    fr=Rotation.from_quat([[float(x[a]) for a in ('qx','qy','qz','qw')] for x in fin])
    lever=np.asarray(audit['T_cpt7_prism'])[:3,3]
    prism=p+rotations.apply(lever);fprism=f+fr.apply(lever)
    last={role:None for role in ('glim_hesai','kimera_front')};firstdel={role:[] for role in last}
    seen=set();ages={role:[] for role in last};stateidx=0;count=0
    with pin(base/'events.csv').open() as ef:
        for row in csv.reader(ef):
            count+=1
            if row[0]=='B':
                role=row[2];end=int(row[9]);arrival=int(row[1]);bid=tuple(row[2:4]+row[6:8])
                last[role]=max(end,last[role] or end)
                if bid not in seen:firstdel[role].append((arrival-end)*1e-9);seen.add(bid)
            if row[0]=='I':
                physical=int(row[2])
                while stateidx<len(sn) and sn[stateidx]<=physical:
                    for role in last:
                        ages[role].append(None if last[role] is None else (int(sn[stateidx])-last[role])*1e-9)
                    stateidx+=1
    assert stateidx==len(sn)
    progress=load_csv(R/'stream_progress.csv')
    assert len(progress)==count
    waits=[];firstwall=None
    with pin(base/'receipts.jsonl').open() as receipts:
        for entry,pr in zip(receipts,progress):
            entry=json.loads(entry)
            assert entry['event']+1==int(pr['event'])
            # Includes service time; subtract measured service to estimate queue wait.
            wait=(int(pr['processed_wall_ns'])-entry['wall_ns'])*1e-9-float(pr['service_seconds'])
            assert wait>=-.005,wait
            waits.append(max(0,wait))
    allage={k:np.array([np.nan if a is None else a for a in v]) for k,v in ages.items()}
    moving_time=(sn-epoch)*1e-9
    use=(moving_time>=4)&(moving_time<=300)
    correction=np.linalg.norm(fprism-prism,axis=1)
    telemetry[mode]=dict(summary=read(R/'summary.json'),
        first_version_arrival_lag_s={k:stats(v) for k,v in firstdel.items()},
        latest_endpoint_age_at_publication_4_to_300s={k:stats(v[use&np.isfinite(v)]) for k,v in allage.items()},
        queue_wait_wall_s=stats(waits),service_wall_s=stats([float(x['service_seconds']) for x in progress]),
        online_to_finalized_prism_displacement_m=stats(correction),
        online_to_finalized_rotation_deg=stats((rotations.inv()*fr).magnitude()*180/np.pi),
        native_path_body_online_m=float(np.linalg.norm(np.diff(p,axis=0),axis=1).sum()),
        native_path_body_finalized_m=float(np.linalg.norm(np.diff(f,axis=0),axis=1).sum()),
        native_path_prism_online_m=float(np.linalg.norm(np.diff(prism,axis=0),axis=1).sum()),
        native_path_prism_finalized_m=float(np.linalg.norm(np.diff(fprism,axis=0),axis=1).sum()),
        online_angular_step_deg=stats((rotations[:-1].inv()*rotations[1:]).magnitude()*180/np.pi),
        finalized_angular_step_deg=stats((fr[:-1].inv()*fr[1:]).magnitude()*180/np.pi))

for path,digest in PINS.items():
    assert sha(path)==digest,path
    if path in sealed:assert sealed[path]==digest,path
save('localization.json',dict(passed=True,created=datetime.datetime.now().astimezone().isoformat(),
    source='Existing sealed CP104 EVO errors and saved alignment; no new scoring',
    common_rows=len(t),curves=details,fusion_OFF_to_ON_error_energy=delta,
    telemetry=telemetry,input_sha256=PINS))
np.savez_compressed(H/'localized_errors.npz',time_s=t,**{key+'_error':value for key,value in vectors.items()})
print(json.dumps(dict(fusion_change=delta,telemetry=telemetry),indent=2))
