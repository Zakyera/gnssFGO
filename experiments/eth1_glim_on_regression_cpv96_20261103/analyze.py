"""Saved-only GLIM repeatability/accuracy diagnosis. No estimator execution."""
import os
os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')
os.environ.setdefault('OMP_NUM_THREADS', '1')
import copy, hashlib, importlib.util, importlib.metadata, io, json, re, sys, zipfile
from pathlib import Path
sys.dont_write_bytecode = True
sys.path.append('/home/yeranis/.local/share/pipx/venvs/rosbags/lib/python3.12/site-packages')
import numpy as np
from evo.core import metrics, trajectory, units, filters
from evo.tools import file_interface
import evo

H = Path(__file__).resolve().parent
E = H.parent
W = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
OLD = E/'eth1_split_imu_persistent_on_cpv91_20261102_attempt1'
NEW = E/'eth1_split_imu_live_pair_cpv95_20261103_continuation1'
SOURCES = dict(old=W/'runs'/OLD.name/'prepared/on_source',
               new=W/'runs/eth1_split_imu_live_pair_cpv95_20261103_attempt3/prepared/full_on_source')
PINS = {}
ANSI = re.compile(r'\x1b\[[0-9;]*m')

def sha(p):
    with Path(p).open('rb') as f: return hashlib.file_digest(f, 'sha256').hexdigest()

def pin(p):
    p = Path(p); PINS[str(p)] = sha(p); return p

def save(name, data):
    with (H/name).open('x') as f: json.dump(data, f, indent=2, allow_nan=False)

def loadmod(path):
    spec = importlib.util.spec_from_file_location('cp96_geometry', pin(path))
    mod = importlib.util.module_from_spec(spec); spec.loader.exec_module(mod); return mod

g = loadmod(E/'fixed_lag_eig2_g2k_reference_fl4y_20261017/analyze.py')
AUDIT = json.loads(pin(SOURCES['new'].parent/'input_audit.json').read_text())
EPOCH = AUDIT['epoch_ns']

def ns(text):
    from decimal import Decimal
    return int(Decimal(str(text))*1_000_000_000)

def stats(error, relation=metrics.PoseRelation.translation_part):
    error = np.asarray(error, float)
    if not len(error): return dict(n=0)
    m = metrics.APE(relation); m.error = error
    return dict(n=len(error), **{k:float(v) for k,v in m.get_all_statistics().items()}, p95=float(np.percentile(error,95)))

def pose_errors(ref, est):
    paths = (trajectory.PosePath3D(poses_se3=ref), trajectory.PosePath3D(poses_se3=est))
    out = {}
    for key, relation in [('translation_m', metrics.PoseRelation.translation_part),('rotation_deg', metrics.PoseRelation.rotation_angle_deg)]:
        m = metrics.APE(relation); m.process_data(paths); out[key] = m.error.copy()
    return out

def ape_archive(path):
    with zipfile.ZipFile(pin(path)) as z:
        return {k:np.load(io.BytesIO(z.read(k+'.npy'))) for k in ['error_array','timestamps','alignment_transformation_sim3']}

def parse_source(folder):
    logs = [p for p in folder.glob('ros_logs/*/glim_rosnode-1-stdout.log') if p.parent.name != 'latest']; assert len(logs)==1
    rows=[]; merges=[]; residuals=[]; transports=[]
    for n,line in enumerate(pin(logs[0]).open(errors='strict'),1):
        def decode(tag): return ANSI.sub('',line.split(tag+',',1)[1]).strip().split(',')
        if 'GLIM_CBS_MEAN_ALIGNMENT_ROW,' in line:
            f=decode('GLIM_CBS_MEAN_ALIGNMENT_ROW'); assert len(f)==12
            rows.append(dict(line=n,source_ids=list(map(int,f[:2])),local_ids=list(map(int,f[2:4])),
                source_ns=[ns(x) for x in f[4:6]],local_ns=[ns(x) for x in f[6:8]],causal_ns=ns(f[8]),
                raw=g.exp(np.asarray(f[9].split(';'),float)),aligned=g.exp(np.asarray(f[10].split(';'),float)),C=g.matrix(f[11])))
        elif 'GLIM_CBS_POSE_MERGE_ROW,' in line:
            f=decode('GLIM_CBS_POSE_MERGE_ROW'); merges.append(dict(update=int(f[0]),edge=f[1],values=list(map(float,f[2:]))))
        elif 'CBS_ODOM_PREINJECTION_RESIDUAL_ROW,' in line:
            f=decode('CBS_ODOM_PREINJECTION_RESIDUAL_ROW'); residuals.append(dict(edge=f[3]+'->'+f[4],values=list(map(float,f[7:]))))
        elif 'GLIM_CBS_SOURCE_TRANSPORT_ROW,' in line:
            f=decode('GLIM_CBS_SOURCE_TRANSPORT_ROW'); transports.append((list(map(int,f[1:3])),g.matrix(f[4]),g.matrix(f[5])))
    assert len(rows)==len(merges)==len(residuals)==len(transports)==1473
    for row,merge,res,trans in zip(rows,merges,residuals,transports):
        edge='x%d->x%d'%tuple(row['local_ids'])
        assert merge['edge']==res['edge']==edge and trans[0]==row['source_ids']
        assert np.max(abs(trans[1]-trans[2]))<1e-14 and np.max(abs(row['C']-trans[2]))<1e-14
        row['merge']=merge; row['residual']=res; row['time_s']=(row['causal_ns']-EPOCH)*1e-9
    return rows

def main():
    assert importlib.metadata.version('evo')=='1.36.3' and not (H/'analysis.json').exists()
    pin(Path(__file__)); pin(H/'PROTOCOL.txt')
    terminal={}
    for label,root in [('old',OLD),('new',NEW)]:
        c=json.loads(pin(root/'complete.json').read_text()); assert c['passed']
        for p,d in c['output_sha256'].items(): assert sha(p)==d,p
        terminal[label]=c['output_sha256']
    refs={}; curves={}; archives={}
    for label,root in [('old',OLD),('new',NEW)]:
        folder=root/'evaluation/common'; refs[label]=file_interface.read_tum_trajectory_file(pin(folder/'reference.tum'))
        for mode in ['off','on']:
            for role in ['glim','kimera']:
                stem=(mode.upper()+'_'+('GLIM' if role=='glim' else 'Kimera')) if label=='old' else mode+'_'+role
                key=label+'_'+mode+'_'+role
                curves[key]=file_interface.read_tum_trajectory_file(pin(folder/(stem+'.tum')))
                archives[key]=ape_archive(folder/(stem+'_ape.zip'))
    assert (OLD/'evaluation/common/reference.tum').read_bytes()==(NEW/'evaluation/common/reference.tum').read_bytes()
    assert (OLD/'evaluation/common/pair_indices.json').read_bytes()==(NEW/'evaluation/common/pair_indices.json').read_bytes()
    pin(OLD/'evaluation/common/pair_indices.json');pin(NEW/'evaluation/common/pair_indices.json')
    ref=refs['old'];t=ref.timestamps
    assert len(t)==2969 and t[-1]<347 and all(np.array_equal(c.timestamps,t) for c in curves.values())
    cuts=np.split(np.arange(len(t)),np.flatnonzero(np.diff(t)>.15)+1)
    errors={k:v['error_array'] for k,v in archives.items()}
    vectors={}; aligned={}; early_anchored={}; bins=[]
    for key,c in curves.items():
        ac=copy.deepcopy(c); ac.transform(archives[key]['alignment_transformation_sim3']);aligned[key]=ac
        check=pose_errors(ref.poses_se3,ac.poses_se3)['translation_m'];assert np.max(abs(check-errors[key]))<1e-9
        vectors[key]=ac.positions_xyz-ref.positions_xyz
        early=copy.deepcopy(c); rr,tt,ss=early.align(ref,n=int(np.sum(t<30)))
        assert abs(ss-1)<1e-12
        early_anchored[key]=pose_errors(ref.poses_se3,early.poses_se3)['translation_m']
    for start in range(0,330,30):
        ix=np.flatnonzero((t>=start)&(t<start+30))
        if not len(ix):continue
        local={}
        for k,c in curves.items():
            cr=copy.deepcopy(c);rf=copy.deepcopy(ref);cr.reduce_to_ids(ix);rf.reduce_to_ids(ix);cr.align(rf)
            local[k]=stats(pose_errors(rf.poses_se3,cr.poses_se3)['translation_m'])
        bins.append(dict(start_s=start,end_s=start+30,rows=len(ix),
            original_global_alignment={k:stats(v[ix]) for k,v in errors.items()},
            independent_window_alignment=local,
            early30s_alignment={k:stats(v[ix]) for k,v in early_anchored.items()},
            GLIM_ON_delta_sse=float(np.sum(errors['new_on_glim'][ix]**2-errors['old_on_glim'][ix]**2)),
            mean_aligned_error_vector={k:np.mean(v[ix],axis=0).tolist() for k,v in vectors.items()}))
    long_rpe={}
    for delta in [.5,5.,10.,20.,50.]:
        out={}
        for key,c in curves.items():
            pooled={'translation_m':[],'rotation_deg':[]}; pairchecks=[]
            for ids in cuts:
                if ref.distances[ids[-1]]-ref.distances[ids[0]]<delta:continue
                cr=copy.deepcopy(c);rf=copy.deepcopy(ref);cr.reduce_to_ids(ids);rf.reduce_to_ids(ids)
                if not filters.filter_pairs_by_path(rf.poses_se3,delta,delta*.1,all_pairs=False):continue
                for name,relation in [('translation_m',metrics.PoseRelation.translation_part),('rotation_deg',metrics.PoseRelation.rotation_angle_deg)]:
                    m=metrics.RPE(relation,delta=delta,delta_unit=units.Unit.meters,pairs_from_reference=True,all_pairs=False)
                    m.process_data((rf,cr));pooled[name].extend(m.error.tolist())
                    if name=='translation_m':pairchecks.extend(t[ids][m.delta_ids].tolist())
            out[key]={name:stats(err) for name,err in pooled.items()};out[key]['pair_end_times']=pairchecks
        assert all(z['pair_end_times']==next(iter(out.values()))['pair_end_times'] for z in out.values())
        long_rpe[str(delta)]=out
    print('Trajectory diagnostics computed',flush=True)
    reference=g.Reference(pin(W/'src/datasets/GrandTour/2024-10-01-11-29-55_cpt7_ie_tc.bag'))
    src={label:parse_source(folder) for label,folder in SOURCES.items()}
    assert [z['source_ids'] for z in src['old']]==[z['source_ids'] for z in src['new']]
    assert [z['source_ns'] for z in src['old']]==[z['source_ns'] for z in src['new']]
    belief={};event_arrays={};events=[]
    for label,rows in src.items():
        arrays={}
        for kind,endpoints in [('raw','source_ns'),('aligned','local_ns')]:
            expected=np.array([g.inv(reference.at(z[endpoints][0]))@reference.at(z[endpoints][1]) for z in rows])
            est=np.array([z[kind] for z in rows]);ee=pose_errors(expected,est)
            signed=np.array([g.log(g.inv(a)@b) for a,b in zip(expected,est)])
            arrays[kind]=dict(**ee,signed_error=signed)
        traces=np.array([[np.trace(z['C'][:3,:3]),np.trace(z['C'][3:,3:])] for z in rows])
        merge=np.array([z['merge']['values'] for z in rows])
        belief[label]=dict(rows=len(rows),raw={k:stats(v) for k,v in arrays['raw'].items() if k!='signed_error'},
            aligned={k:stats(v) for k,v in arrays['aligned'].items() if k!='signed_error'},
            trace_medians=dict(rotation_rad2=float(np.median(traces[:,0])),translation_m2=float(np.median(traces[:,1]))),
            residual_to_external=dict(before_rotation_rad=stats(merge[:,0]),before_translation_log_m=stats(merge[:,1]),
                after_rotation_rad=stats(merge[:,2]),after_translation_log_m=stats(merge[:,3])))
        event_arrays[label]=arrays
    for i,(a,b) in enumerate(zip(src['old'],src['new'])):
        changed=a['local_ids']!=b['local_ids']
        row=dict(source_ids=a['source_ids'],source_times_s=[(x-EPOCH)*1e-9 for x in a['source_ns']],binding_changed=changed)
        for label,z in [('old',a),('new',b)]:
            ar=event_arrays[label]
            row[label]=dict(line=z['line'],time_s=z['time_s'],local_ids=z['local_ids'],local_times_s=[(x-EPOCH)*1e-9 for x in z['local_ns']],
                raw_error={k:float(ar['raw'][k][i]) for k in ['translation_m','rotation_deg']},
                aligned_error={k:float(ar['aligned'][k][i]) for k in ['translation_m','rotation_deg']},
                raw_signed_error=ar['raw']['signed_error'][i].tolist(),aligned_signed_error=ar['aligned']['signed_error'][i].tolist(),
                trace_rotation_rad2=float(np.trace(z['C'][:3,:3])),trace_translation_m2=float(np.trace(z['C'][3:,3:])),merge=z['merge'])
        events.append(row)
    belief_bins=[]
    for start in range(0,330,30):
        ix=np.array([i for i,z in enumerate(events) if start<=z['source_times_s'][1]<start+30],int)
        if not len(ix):continue
        row=dict(start_s=start,rows=len(ix),changed_bindings=sum(events[i]['binding_changed'] for i in ix))
        for label in src:
            row[label]={kind:{k:stats(event_arrays[label][kind][k][ix]) for k in ['translation_m','rotation_deg']} for kind in ['raw','aligned']}
            row[label]['aligned_signed_mean']=np.mean(event_arrays[label]['aligned']['signed_error'][ix],axis=0).tolist()
        belief_bins.append(row)
    print('Belief diagnostics computed',flush=True)
    # Static scientific figures, no trajectory smoothing or replacement scoring.
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,ax=plt.subplots(3,1,figsize=(13,10),sharex=True)
    for key,color in [('old_on_glim','#4477aa'),('new_on_glim','#cc6677')]:
        ax[0].plot(t,errors[key]*100,label=key.replace('_',' '),lw=1,color=color)
        ax[1].plot(t,np.cumsum(errors[key]**2),label=key.replace('_',' '),color=color)
    for label,color in [('old','#4477aa'),('new','#cc6677')]:
        times=[z['source_times_s'][1] for z in events]
        ax[2].plot(times,event_arrays[label]['aligned']['translation_m']*100,label=label+' aligned K→G',lw=.7,color=color,alpha=.8)
    for z in events:
        if z['binding_changed']:
            for a in ax:a.axvline(z['source_times_s'][1],color='grey',alpha=.17,lw=.7)
    for a in ax:a.legend();a.grid(alpha=.2)
    ax[0].set_ylabel('Global-aligned EVO APE (cm)');ax[1].set_ylabel('Cumulative squared APE (m²)');ax[2].set_ylabel('Aligned belief error (cm)');ax[2].set_xlabel('Sequence time (s)')
    fig.suptitle('CP91 vs CP95: saved GLIM ON and actual K→G beliefs\nGrey markers: different local edge bindings; association is not causation')
    fig.tight_layout();fig.savefig(H/'diagnostics.png',dpi=150);plt.close(fig)
    summary=dict(passed=True,task='CP-V96',evo_version=evo.__version__,rows=len(t),common_mask_identical=True,
        original_scores={k:stats(v) for k,v in errors.items()},time_bins=bins,long_baseline_RPE=long_rpe,
        belief_summary=belief,belief_time_bins=belief_bins,binding_differences=[z for z in events if z['binding_changed']],
        reference_max_bracket_ns=reference.max_bracket_ns,
        methodology=['APE bins retain each original full-window alignment; not causal onset by themselves.',
          'Window-aligned and first30s-aligned APE are explicitly supplemental diagnostics.',
          'Long-distance RPE uses EVO reference-distance non-overlapping pairs within existing contiguous segments.',
          'Belief pose errors use EVO on native CPT7 relative transforms at their exact endpoints, not 0.5m RPE.',
          'Residual-to-external rows are native logged Logmap norms; not reference errors or covariance calibration.',
          'No optimizer counterfactual, no proof that load/binding differences cause regression; IE-TC shares CPT7.'])
    for p,d in PINS.items():assert sha(p)==d,p
    for pins in terminal.values():
        for p,d in pins.items():assert sha(p)==d,p
    save('analysis.json',summary);save('belief_events.json',events);save('input_sha256.json',PINS)
    save('verification.json',dict(passed=True,input_files=len(PINS),prior_terminal_counts={k:len(v) for k,v in terminal.items()},
        output_sha256={str(H/n):sha(H/n) for n in ['analysis.json','belief_events.json','diagnostics.png','input_sha256.json']},
        source_runs=0,receiver_replays=0,optimizer_runs=0,original_artifacts_unchanged=True))
    print(json.dumps(dict(scores=summary['original_scores'],belief_summary=belief,binding_differences=len(summary['binding_differences'])),indent=2))

if __name__=='__main__':main()
