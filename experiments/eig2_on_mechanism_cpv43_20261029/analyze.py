"""Saved-data geometry/lifecycle diagnosis. No covariance analysis or estimator."""
import csv
import hashlib
import json
import re
from collections import Counter
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

H = Path(__file__).resolve().parent
E = H.parent
W = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
P = W/'runs/eig2_refresh_full_cpv42_20261029/prepared'
C = W/'runs/kimera_on_incremental_refresh_cpv40_20261029'
H42 = E/'eig2_refresh_full_cpv42_20261029'
EPOCH = 1730638816563546034
PINS = {}
ANSI = re.compile(r'\x1b\[[0-9;]*m')

def sha(p):
    with Path(p).open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()

def pin(p):
    p = Path(p).resolve()
    PINS[str(p)] = sha(p)
    return p

def save(name, data):
    with (H/name).open('x') as f:
        json.dump(data, f, indent=2, allow_nan=False)

def stat(a):
    a = np.asarray(a, float)
    assert np.isfinite(a).all()
    return dict(n=len(a), median=float(np.median(a)), p95=float(np.percentile(a,95)),
                maximum=float(a.max()), minimum=float(a.min())) if len(a) else dict(n=0)

def skew(w):
    x,y,z = w
    return np.array([[0,-z,y],[z,0,-x],[-y,x,0]])

def exp(v):
    v = np.asarray(v, float)
    w = v[:3]; theta = np.linalg.norm(w); K = skew(w)
    if theta < 1e-6:
        J = np.eye(3)+(0.5-theta**2/24)*K+(1/6-theta**2/120)*(K@K)
    else:
        J = np.eye(3)+(1-np.cos(theta))/theta**2*K+(theta-np.sin(theta))/theta**3*(K@K)
    T = np.eye(4); T[:3,:3] = Rotation.from_rotvec(w).as_matrix(); T[:3,3] = J@v[3:]
    return T

def log(T):
    w = Rotation.from_matrix(T[:3,:3]).as_rotvec(); theta = np.linalg.norm(w); K = skew(w)
    c = 1/12+theta**2/720 if theta < 1e-6 else (1-theta/2/np.tan(theta/2))/theta**2
    return np.r_[w, (np.eye(3)-0.5*K+c*(K@K))@T[:3,3]]

def inv(T):
    out = np.eye(4); out[:3,:3] = T[:3,:3].T; out[:3,3] = -out[:3,:3]@T[:3,3]
    return out

def pose(f):
    v = np.array(f,float); T = np.eye(4); T[:3,3] = v[:3]
    T[:3,:3] = Rotation.from_quat(v[[4,5,6,3]]).as_matrix()
    return T

def diff(a,b):
    v = log(inv(a)@b)
    return dict(rotation_deg=float(np.rad2deg(np.linalg.norm(v[:3]))),
                translation_log_m=float(np.linalg.norm(v[3:])),
                translation_m=float(np.linalg.norm(a[:3,3]-b[:3,3])))

def parse_source(path):
    windows={}; aligned={}; frozen={}; merges={}; retired=[]; rowlines={}
    tags=('KIMERA_CBS_REVISION_EDGE_ROW','KIMERA_CBS_MEAN_ALIGNMENT_ROW',
          'FL4M_FROZEN_FACTOR_ROW','KIMERA_CBS_POSE_MERGE_ROW','KIMERA_CBS_REVISION_POSE_RETIRE_ROW')
    for n,line in enumerate(pin(path).open(),1):
        for tag in tags:
            if tag+',' not in line: continue
            f=ANSI.sub('',line.split(tag+',',1)[1]).strip().split(','); k=int(f[0])
            if tag==tags[0]:
                assert len(f)==64
                d=windows.setdefault(k,{})
                for i,raw in [(int(f[2]),f[8:15]),(int(f[3]),f[15:22])]:
                    T=pose(raw)
                    if i in d: assert np.array_equal(d[i],T)
                    d[i]=T
                rowlines.setdefault(k,n)
            elif tag==tags[1]:
                key=(k,int(f[3]),int(f[4]))
                assert len(f)==13 and key not in aligned
                aligned[key]=dict(ids=list(map(int,f[1:5])), times=list(map(int,f[5:10])),
                                raw=exp(f[10].split(';')), mean=exp(f[11].split(';')), line=n)
            elif tag==tags[2]:
                key=(k,int(f[3]),int(f[4])); assert key not in frozen
                frozen[key]=dict(ids=list(map(int,f[1:5])), mean=exp(f[5].split(';')), line=n)
            elif tag==tags[3]:
                assert k not in merges
                merges[k]=dict(edge=f[1], values=list(map(float,f[2:6])), line=n)
            else: retired.append(dict(k=k, key=int(f[1]), line=n))
    return dict(windows=windows, aligned=aligned, frozen=frozen, merges=merges, retired=retired, lines=rowlines)

def parse_stdout(path):
    current=None; pre={}; post={}; life={}; refresh={}; actions=[]; failure=[]
    for n,line in enumerate(pin(path).open(),1):
        if 'CBS_BPSAM_ACTIVE_WINDOW_LIFECYCLE_ROW,' in line:
            f=line.split('CBS_BPSAM_ACTIVE_WINDOW_LIFECYCLE_ROW,',1)[1].strip().split(',')
            assert f[0]=='k'; current=int(float(f[1])); assert current not in life
            life[current]=dict(lag=float(f[2]), active_keys=int(f[3]), marginal_keys=int(f[4]),
                               active_factors=int(f[5]), retiring_factors=int(f[6]), line=n)
        for tag,dest in [('CBS_ODOM_PREINJECTION_RESIDUAL_ROW',pre),('CBS_ODOM_TEMPORARY_POSTSOLVE_RESIDUAL_ROW',post)]:
            if tag+',' not in line: continue
            f=line.split(tag+',',1)[1].strip().split(',')
            key=(current,int(f[3][1:]),int(f[4][1:]))
            assert f[:3]==['G2K','k','g'] and key not in dest
            dest[key]=dict(edge=f[3]+'->'+f[4], rotation_deg=float(np.rad2deg(float(f[8]))),
                               translation_log_m=float(f[9]), source=f[6], line=n)
        if 'CPK9_PRIOR_REFRESH,' in line:
            f=line.split('CPK9_PRIOR_REFRESH,',1)[1].strip().split(',')
            k=int(float(f[0])); assert k==current
            refresh[k]=dict(removals=int(f[1]), priors=int(f[2]), released_keys=int(f[3]), line=n)
        if 'CBS_BPSAM_ACTIVE_WINDOW_RECEIVER_EDGE_ROW,' in line:
            f=line.split('CBS_BPSAM_ACTIVE_WINDOW_RECEIVER_EDGE_ROW,',1)[1].strip().split(',')
            actions.append(dict(k=current,edge=f[2],action=f[3],details=f[4:],line=n))
        if re.search(r'rollback|retry_local_update|retry_without|Indeterminant|indeterminant|CPV40.*fail|staged_factor_dropped',line):
            failure.append(dict(k=current,line=n,text=line.strip()))
    return dict(pre=pre,post=post,life=life,refresh=refresh,actions=actions,failures=failure)

def main():
    assert not (H/'analysis.json').exists()
    pin(__file__); pin(H/'PROTOCOL.txt')
    originals=json.loads(pin(H42/'complete.json').read_text())['output_sha256']
    reportpins=json.loads(pin(H42/'evaluation/input_sha256.json').read_text())
    protected=json.loads(pin(P/'protected_manifest.json').read_text())
    existing=json.loads(pin(H42/'evaluation/revision_events.json').read_text())
    code=[C/'BpsamOnRefresh.inc',C/'OriginalChartPrior.h',C/'RetirementAudit.h',
          C/'VioBackend.cpp',C/'include/cbs/bpsam/incremental_fixed_lag_bpsam_smoother.h',
          W/'src/cbs/src/bpsam/bpsam.cpp',
          W/'src/Kimera-VIO/include/kimera-vio/imu-frontend/CbsNativeImuMeanBridge.h']
    for p in code: pin(p)
    data={}; ts={}; costs={}; out={}; stdout={}
    for mode in ('off','on'):
        s=P/f'full_{mode}_source'; data[mode]=parse_source(s/'estimators.log')
        rr=list(csv.DictReader(pin(s/'kimera_odometry.csv').open()))
        ts[mode]=[int(r['field.header.stamp']) for r in rr if r.get('field.header.seq') and r['field.header.seq'].isdigit()]
        costs[mode]=np.loadtxt(pin(s/'objective.csv'),delimiter=',')
        assert np.array_equal(costs[mode][:,0],np.arange(len(ts[mode])))
        logs=[p for p in (s/'ros_logs').glob('*/kimera_vio_ros-kimera_vio_ros_node-4-stdout.log') if p.parent.name!='latest']
        assert len(logs)==1; stdout[mode]=parse_stdout(logs[0])
        assert len(stdout[mode]['life'])==len(ts[mode])
    on=data['on']; off=data['off']; native=stdout['on']; rows=[]; missing=[]; parity=[]
    assert set(on['aligned'])==set(on['frozen'])==set(native['pre'])==set(native['post'])
    for key,d in on['aligned'].items():
        k=key[0]
        assert np.max(abs(exp(log(d['raw']))-d['raw']))<1e-10
        assert np.array_equal(d['mean'],on['frozen'][key]['mean']) and d['ids']==on['frozen'][key]['ids']
        si,sj,i,j=d['ids']; sf,st,rf,rt,causal=d['times']
        assert [rf,rt,causal]==[ts['on'][i],ts['on'][j],ts['on'][k]]
        assert sf<st and rf<rt and max(sf,st,rf,rt)<=causal
        assert abs(sf-rf)<=50_000_000 and abs(st-rt)<=50_000_000
        edge=f'x{i}->x{j}'; assert edge==native['pre'][key]['edge']==native['post'][key]['edge']
        row=dict(k=k,time_s=(causal-EPOCH)*1e-9, sender=[si,sj],receiver=[i,j],
                 sender_duration_s=(st-sf)*1e-9,receiver_duration_s=(rt-rf)*1e-9,
                 endpoint_offsets_ms=[(sf-rf)*1e-6,(st-rt)*1e-6],
                 alignment_correction=diff(d['raw'],d['mean']),
                 mean_motion_m=float(np.linalg.norm(d['mean'][:3,3])),
                 mean_rotation_deg=float(np.rad2deg(Rotation.from_matrix(d['mean'][:3,:3]).magnitude())),
                 pre=native['pre'][key],ordinary_post=native['post'][key],lifecycle=native['life'][k],
                 total_cost_pre=float(costs['on'][k,4]),total_cost_post=float(costs['on'][k,5]),
                 imu_cost_pre=float(costs['on'][k,6]),imu_cost_post=float(costs['on'][k,7]),
                 source_line=d['line'])
        for tag,windows,idx in [('pre_geometry',on['windows'],k-1),('final',on['windows'],k)]:
            if i not in windows.get(idx,{}) or j not in windows[idx]:
                missing.append(dict(k=k,phase=tag,edge=edge)); continue
            relative=inv(windows[idx][i])@windows[idx][j]
            row[tag]=diff(d['mean'],relative)
        if 'pre_geometry' in row:
            delta=max(abs(row['pre_geometry']['rotation_deg']-row['pre']['rotation_deg']),
                      abs(row['pre_geometry']['translation_log_m']-row['pre']['translation_log_m']))
            parity.append(delta); assert delta<1e-7,(k,delta)
        if 'final' in row and k in on['merges'] and on['merges'][k]['edge']==edge:
            merge=on['merges'][k]
            assert abs(np.rad2deg(merge['values'][2])-row['final']['rotation_deg'])<.0001
            assert abs(merge['values'][3]-row['final']['translation_log_m'])<.00001
        # OFF comparison only where exact keyframe timestamps match, no interpolation.
        if k<len(ts['off']) and ts['off'][k]==ts['on'][k] and all(z<len(ts['off']) and ts['off'][z]==ts['on'][z] for z in [i,j]):
            if i in off['windows'].get(k,{}) and j in off['windows'][k]:
                row['corrected_off_same_edge']=diff(d['mean'],inv(off['windows'][k][i])@off['windows'][k][j])
        rows.append(row)
    shape=[]
    for mode in ('off','on'):
        ww=data[mode]['windows']; modeevents={e['k']:e for e in existing[mode]}
        out[mode]=dict(first_marginalization=next((dict(k=k,**v) for k,v in stdout[mode]['life'].items() if v['marginal_keys']),None),
                       first_peer_retirement=next((dict(k=k,**v) for k,v in stdout[mode]['life'].items() if v['retiring_factors']),None),
                       first_prior_refresh=next((dict(k=k,**v) for k,v in stdout[mode]['refresh'].items()),None),
                       recovery_events=stdout[mode]['failures'],action_counts=dict(Counter(x['action'] for x in stdout[mode]['actions'])),
                       imu_cost_first_above={str(b):next((dict(k=k,time_s=(ts[mode][k]-EPOCH)*1e-9,cost=float(c[7])) for k,c in enumerate(costs[mode]) if c[7]>b),None) for b in [1,10,100,1000]},
                       total_cost_increases=[int(k) for k,c in enumerate(costs[mode]) if c[5]>c[4]+1e-7])
        out[mode]['shape_missing_previous_snapshot_ids']=[]
        for k,e in modeevents.items():
            if k not in ww or k-1 not in ww or k-1 not in ww[k] or k-1 not in ww[k-1]:
                out[mode]['shape_missing_previous_snapshot_ids'].append(k)
                continue
            keys=sorted(set(ww[k])&set(ww[k-1]))
            if len(keys)<3:continue
            old=np.array([ww[k-1][i][:3,3] for i in keys]); new=np.array([ww[k][i][:3,3] for i in keys])
            dr=np.array([ww[k][i][:3,:3]@ww[k-1][i][:3,:3].T for i in keys])
            r=Rotation.from_matrix(dr).mean().as_matrix(); tr=new.mean(0)-r@old.mean(0)
            residual=new-(old@r.T+tr); energy=float(np.sum((new-old)**2))
            den=np.sum((old-old.mean(0))**2)
            change=diff(ww[k-1][k-1],ww[k][k-1])
            assert abs(change['rotation_deg']-e['revision_rotation_deg'])<1e-7
            assert abs(change['translation_m']-e['previous_revision_m'])<1e-8
            shape_max=float(np.linalg.norm(residual,axis=1).max())
            assert abs(shape_max-e['shape_max_m'])<1e-8
            shape.append(dict(mode=mode,**e,overlap=len(keys),rigid_rotation_deg=float(np.rad2deg(Rotation.from_matrix(r).magnitude())),
                              rigid_tilt_deg=float(np.rad2deg(np.arccos(np.clip(r[2,2],-1,1)))),
                              rigid_yaw_deg=float(np.rad2deg(np.arctan2(r[1,0],r[0,0]))),
                              rigid_explained_fraction=1-float(np.sum(residual**2))/energy if energy>1e-20 else None,
                              independently_reconstructed_shape_max_m=shape_max,
                              extent_ratio=float(np.sqrt(np.sum((new-new.mean(0))**2)/den)) if den>1e-12 else None))
    for mode in ('off','on'):
        ss=[x for x in shape if x['mode']==mode and x['previous_revision_m']>.5]
        out[mode]['large_revision_shapes']={key:stat([x[key] for x in ss if x[key] is not None]) for key in ['rigid_explained_fraction','shape_max_m','extent_ratio','rigid_yaw_deg','rigid_tilt_deg']}
    paired=[x for x in rows if 'final' in x]
    phase_summary={phase:{key:stat([x[phase][key] for x in paired]) for key in ['rotation_deg','translation_log_m']} for phase in ['pre','ordinary_post','final']}
    phase_summary['refresh_rotation_improves']=sum(x['final']['rotation_deg']<x['ordinary_post']['rotation_deg']-1e-9 for x in paired)
    phase_summary['refresh_translation_improves']=sum(x['final']['translation_log_m']<x['ordinary_post']['translation_log_m']-1e-9 for x in paired)
    phase_summary['count']=len(paired)
    selected=[6,10,20,21,22,23,24,25,26,30,161,254,1550]
    selected=sorted(set(selected+[int(np.argmax(costs['on'][:,7]))]))
    selected_rows=[dict(k=k,off_cost=costs['off'][k].tolist() if k<len(ts['off']) and ts['off'][k]==ts['on'][k] else None,
                        on_cost=costs['on'][k].tolist(),factor=next((r for r in rows if r['k']==k),None),
                        windows=[s for s in shape if s['k']==k]) for k in selected]
    receipt={}; unmatched=[]
    for path,actual in PINS.items():
        expected=originals.get(path,reportpins.get(path,protected.get(path)))
        if expected is not None: assert expected==actual,path;receipt[path]=expected
        else: unmatched.append(path)
    assert all(sha(p)==s for p,s in PINS.items())
    result=dict(scope='saved geometry/lifecycle only; covariance excluded; no optimizer or accuracy evaluation',
                modes=out,candidate_count=len(rows),phase_summary=phase_summary,missing_geometry=missing,
                pre_geometry_log_parity_max=max(parity),matched_previous_pins=len(receipt),inputs_without_prior_pin=unmatched,
                input_sha256=PINS,inputs_unchanged=True,selected=selected_rows,
                limits=['No complete CP-V42 source graph/PIM/velocity/bias checkpoint retained.',
                        'Ordinary-step endpoint residual is logged; whole-state/IMU cost between ordinary and extra refresh is not.',
                        'A post-refresh native weighted IMU cost is not unweighted physical error or a convergence certificate.'])
    save('edge_events.json',rows); save('window_events.json',shape);save('analysis.json',result)
    print(json.dumps({k:v for k,v in result.items() if k not in ['selected','input_sha256','inputs_without_prior_pin']},indent=2))
    print('SELECTED')
    for s in selected_rows:
        f=s['factor']; ev=next((e for e in s['windows'] if e['mode']=='on'),{})
        print(s['k'], 'imu pre/post',s['on_cost'][6:8], 'factor phase rot/trans',
              [(p,f[p]['rotation_deg'],f[p]['translation_log_m']) for p in ['pre','ordinary_post','final'] if f and p in f],
              'revision/tilt/yaw/shape', [ev.get(x) for x in ['previous_revision_m','rigid_tilt_deg','rigid_yaw_deg','shape_max_m']])

if __name__=='__main__':main()
