"""Supplemental saved-data diagnosis; never starts or modifies an estimator."""
import csv
import hashlib
import json
import re
import sys
from pathlib import Path

sys.dont_write_bytecode = True
sys.path.append('/home/yeranis/.local/share/pipx/venvs/rosbags/lib/python3.12/site-packages')
import numpy as np
import yaml
from rosbags.highlevel import AnyReader
from scipy.signal import butter, sosfiltfilt
from scipy.spatial.transform import Rotation as R

H = Path(__file__).resolve().parent
E = H.parent
W = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
N85 = 'eth1_split_imu_off60_cpv85_20261101_attempt1'
N86 = 'eth1_cpt7_control_off60_cpv86_20261102'
P85, P86 = [W/'runs'/n/'prepared' for n in (N85, N86)]
PINS = {}
ANSI = re.compile(r'\x1b\[[0-9;]*m')

def sha(p):
    with Path(p).open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()

def pin(p):
    p = Path(p).resolve()
    PINS[str(p)] = sha(p)
    return p

def load(p):
    return json.loads(pin(p).read_text())

def yload(p):
    return yaml.safe_load(pin(p).read_text().replace('%YAML:1.0', ''))

def save(p, value):
    with p.open('x') as f:
        json.dump(value, f, indent=2, allow_nan=False)

def stats(a):
    a = np.asarray(a)
    return dict(n=len(a), median=float(np.median(a)), p95=float(np.percentile(a, 95)),
                maximum=float(np.max(a)), mean=float(np.mean(a)))

def pose(f):
    return np.asarray(f[:3], float), R.from_quat(np.asarray(f[3:7], float)[[1,2,3,0]]).as_matrix()

def events(p, epoch):
    s = p/'off_source'
    with pin(s/'kimera_odometry.csv').open() as f:
        pubs = list(csv.DictReader(f))
    ts = np.array([int(r['field.header.stamp']) for r in pubs], dtype=np.int64)
    windows = {}
    absolute = {}
    lines = {}
    with pin(s/'estimators.log').open() as f:
        for no, line in enumerate(f, 1):
            tag = 'KIMERA_BACKEND_POSE_PATH_ROW,'
            if tag in line:
                a = ANSI.sub('', line.split(tag, 1)[1]).strip().split(',')
                absolute[int(a[1])] = pose(a[2:9])
            tag = 'KIMERA_CBS_REVISION_EDGE_ROW,'
            if tag not in line:
                continue
            a = ANSI.sub('', line.split(tag, 1)[1]).strip().split(',')
            assert len(a) == 64
            k = int(a[0]); out = windows.setdefault(k, {}); lines.setdefault(k, no)
            for i, data in [(int(a[2]), a[8:15]), (int(a[3]), a[15:22])]:
                pp, rr = pose(data)
                if i in out:
                    assert np.array_equal(out[i][0], pp) and np.array_equal(out[i][1], rr)
                out[i] = (pp, rr)
    costs = np.loadtxt(pin(s/'objective.csv'), delimiter=',')
    assert len(costs) == len(ts) == len(absolute)
    rows = []; missing = []
    for k in range(1, len(ts)):
        if k not in windows or k-1 not in windows[k] or k not in windows[k]:
            missing.append(k); continue
        pp, rr = windows[k][k-1]; new, nr = windows[k][k]
        old, rold = windows.get(k-1, {}).get(k-1, absolute[k-1])
        rows.append(dict(k=k, time_s=float((ts[k]-epoch)*1e-9),
                         previous_revision_m=float(np.linalg.norm(pp-old)),
                         absolute_step_m=float(np.linalg.norm(new-old)),
                         same_update_motion_m=float(np.linalg.norm(new-pp)),
                         revision_rotation_deg=float(np.rad2deg(R.from_matrix(rold.T@rr).magnitude())),
                         postsolve_total_cost=float(costs[k,5]), postsolve_imu_cost=float(costs[k,7]),
                         log_line=lines[k]))
    summary = dict(rows=len(ts), child_frames=sorted({r['field.child_frame_id'] for r in pubs}),
                   missing_revision_ids=missing,
                   previous_revision_m=stats([r['previous_revision_m'] for r in rows]),
                   postsolve_imu_cost=stats(costs[:,7]),
                   first_crossings={str(th):next((r for r in rows if r['previous_revision_m']>th), None)
                                    for th in (.05,.1,.5,1.,5.,10.)})
    summary['time_bins'] = []
    for lo, hi in [(0,5),(5,10),(10,20),(20,40),(40,60)]:
        a=[r for r in rows if lo<=r['time_s']<hi]
        summary['time_bins'].append(dict(start_s=lo,end_s=hi,
             revision_m=stats([r['previous_revision_m'] for r in a]),
             imu_cost=stats([r['postsolve_imu_cost'] for r in a])))
    return rows, summary, ts

def raw_imu(path, epoch):
    rows=[]
    with AnyReader([pin(path)]) as reader:
        cs=[c for c in reader.connections if c.topic.endswith('/imu')]
        for c,t,data in reader.messages(connections=cs, start=epoch, stop=epoch+60*10**9):
            m=reader.deserialize(data,c.msgtype)
            stamp=m.header.stamp.sec*10**9+m.header.stamp.nanosec
            rows.append([(stamp-epoch)*1e-9,m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z,
                         m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z])
    a=np.asarray(rows);assert np.isfinite(a).all() and (np.diff(a[:,0])>0).all()
    return a

def main():
    assert not (H/'analysis.json').exists()
    audit=load(P85/'input_audit.json'); epoch=audit['epoch_ns']
    result=dict(task='CP-V87', scope='saved-data diagnostic, no estimator execution or scoring', epoch_ns=epoch)
    aevents,asummary,ats=events(P85,epoch);cevents,csummary,cts=events(P86,epoch)
    assert np.array_equal(ats,cts)
    result['same_native_keyframe_timestamps']=True
    result['Alphasense_Kimera']=asummary;result['CPT7_Kimera']=csummary
    save(H/'revision_events.json', dict(Alphasense=aevents,CPT7=cevents))
    result['saved_EVO_control']=load(E/N86/'results.json')['common']
    result['profile_differences']=load(E/N86/'profile_parity.json')['differences']
    params={n:yload(p/'off_source/kimera_profile/ImuParams.yaml') for n,p in [('Alpha',P85),('CPT7',P86)]}
    result['noise_parameters']=params
    result['bias_sigma_0p2s']={n:dict(acc=float(np.sqrt(.2)*v['accelerometer_random_walk']),
                                      gyro=float(np.sqrt(.2)*v['gyroscope_random_walk'])) for n,v in params.items()}
    result['bias_between_information_ratio_Alpha_over_CPT7']={
        'acc':(params['CPT7']['accelerometer_random_walk']/params['Alpha']['accelerometer_random_walk'])**2,
        'gyro':(params['CPT7']['gyroscope_random_walk']/params['Alpha']['gyroscope_random_walk'])**2}
    A=raw_imu(audit['bag_files']['alphasense_imu']['path'],epoch)
    C=raw_imu(audit['bag_files']['cpt7_imu']['path'],epoch)
    RAC=np.asarray(audit['T_alphasense_cpt7'])[:3,:3]
    tAC=np.asarray(audit['T_alphasense_cpt7'])[:3,3]
    rCA=-RAC.T@tAC
    result['lever_arm_CPT_to_Alpha_in_CPT_m']=rCA.tolist()
    result['raw']={}
    for name,data in [('Alpha',A),('CPT7',C)]:
        quiet=data[(data[:,0]>=.2)&(data[:,0]<3.0)]
        mean=quiet[:,4:].mean(axis=0); norm=np.linalg.norm(mean)
        # Kimera AlignGravityVectors uses identity when |1-cos(theta)|<1e-3.
        cos=-mean[2]/norm
        result['raw'][name]=dict(samples=len(data),max_gap_s=float(np.diff(data[:,0]).max()),
            quiet_0p2_to_3s_acc_mean=mean.tolist(),quiet_acc_std=quiet[:,4:].std(axis=0).tolist(),
            quiet_acc_norm=float(norm),quiet_gyro_mean=quiet[:,1:4].mean(axis=0).tolist(),
            gravity_direction_angle_deg=float(np.rad2deg(np.arccos(cos))),
            near_gravity_identity_branch=bool(abs(1-cos)<1e-3),
            implied_bias_if_initialized_on_this_quiet_interval=(mean+np.array([0,0,9.81])).tolist(),
            raw_acc_min=data[:,4:].min(axis=0).tolist(),raw_acc_max=data[:,4:].max(axis=0).tolist(),
            raw_acc_extreme_samples=[dict(time_s=float(row[0]),acc=row[4:].tolist())
                                    for row in data[np.argsort(np.linalg.norm(data[:,4:],axis=1))[-5:]]])
    grid=np.arange(.2,59.7,.01)
    ai=np.column_stack([np.interp(grid,A[:,0],A[:,j]) for j in range(1,7)])
    ci=np.column_stack([np.interp(grid,C[:,0],C[:,j]) for j in range(1,7)])
    result['rigid_body_consistency']=[]
    # Identical offline filtering is diagnostic only, never applied to estimator inputs.
    for hz in (2.,5.,10.):
        af=sosfiltfilt(butter(4,hz,fs=100,output='sos'),ai,axis=0)
        cf=sosfiltfilt(butter(4,hz,fs=100,output='sos'),ci,axis=0)
        quiet=(grid>.5)&(grid<2.5);valid=(grid>5)&(grid<58)
        omega=cf[:,:3]-cf[quiet,:3].mean(axis=0)
        omega_dot=np.gradient(omega,.01,axis=0)
        lever=np.cross(omega_dot,rCA)+np.cross(omega,np.cross(omega,rCA))
        pred=(cf[:,3:]+lever)@RAC.T
        observed=af[:,3:]
        bias=(observed-pred)[quiet].mean(axis=0)
        residual=observed-pred-bias
        gyro_a=af[:,:3]-af[quiet,:3].mean(axis=0)
        gyro_c=(cf[:,:3]-cf[quiet,:3].mean(axis=0))@RAC.T
        row=dict(lowpass_hz=hz,quiet_bias_difference_m_s2=bias.tolist(),
            acc_residual_norm_m_s2=stats(np.linalg.norm(residual[valid],axis=1)),
            lever_acceleration_norm_m_s2=stats(np.linalg.norm(lever[valid],axis=1)),
            gyro_residual_norm_rad_s=stats(np.linalg.norm((gyro_a-gyro_c)[valid],axis=1)),
            gyro_axis_correlations=[float(np.corrcoef(gyro_a[valid,j],gyro_c[valid,j])[0,1]) for j in range(3)],
            acc_axis_correlations=[float(np.corrcoef(observed[valid,j],pred[valid,j])[0,1]) for j in range(3)])
        row['heldout_scalar_acc_scale_fits']=[]
        for lo,hi,tlo,thi in [(5,30,30,58),(30,58,5,30)]:
            train=(grid>=lo)&(grid<hi);test=(grid>=tlo)&(grid<thi)
            x=pred[train]; y=observed[train]
            x0=x-x.mean(axis=0);y0=y-y.mean(axis=0)
            scale=np.sum(x0*y0)/np.sum(x0*x0); intercept=y.mean(axis=0)-scale*x.mean(axis=0)
            fitted=scale*pred+intercept
            row['heldout_scalar_acc_scale_fits'].append(dict(train=[lo,hi],test=[tlo,thi],scale=float(scale),
                offset_m_s2=intercept.tolist(),heldout_residual_norm_m_s2=stats(np.linalg.norm((observed-fitted)[test],axis=1))))
        result['rigid_body_consistency'].append(row)
    for p in [W/'src/Kimera-VIO/src/initial/InitializationFromImu.cpp',
              W/'src/Kimera-VIO/src/utils/UtilsOpenCV.cpp',
              W/'src/Kimera-VIO/src/imu-frontend/ImuFrontend.cpp',
              W/'src/Kimera-VIO-ROS/src/RosOnlineDataProvider.cpp',
              W/'runs/persistent_compatibility_cpv76_20261101/VioBackend.cpp',
              H/'PROTOCOL.txt',Path(__file__)]:pin(p)
    assert all(sha(p)==s for p,s in PINS.items())
    save(H/'input_sha256.json',PINS)
    save(H/'analysis.json',result)
    print(json.dumps(result,indent=2))

if __name__=='__main__':main()
