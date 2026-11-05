"""Read-only MAIN-window and publication-change diagnosis; no optimizer runs."""
import hashlib
import importlib.util
import json
import re
import sys
from pathlib import Path
sys.dont_write_bytecode = True
import numpy as np
from scipy.spatial.transform import Rotation as R

H = Path(__file__).resolve().parent
E = H.parent
H27 = E/'newest_absolute_comparison_cpv27_20261028'
H32 = E/'all_evo_tables_cpv32_20261029'
spec=importlib.util.spec_from_file_location('cpv27', H27/'analyze.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
a=m.a
PINS={}
ANSI=re.compile(r'\x1b\[[0-9;]*m')

def sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        while b:=f.read(8*1024*1024): h.update(b)
    return h.hexdigest()

def pin(path,expected=None):
    path=Path(path).resolve();s=sha(path)
    if expected is not None: assert s==expected,path
    PINS[str(path)]=s
    return path

def load(path): return json.loads(pin(path).read_text())
def save(path,data):
    with Path(path).open('x') as f: json.dump(data,f,indent=2)
def stats(values):
    x=np.asarray(values,float)
    return dict(count=len(x),median=float(np.median(x)),p95=float(np.percentile(x,95)),
                maximum=float(x.max()),mean=float(x.mean())) if len(x) else dict(count=0)

def pose(raw):
    p=np.asarray(raw[:3],float);q=np.asarray(raw[3:7],float)
    return p,R.from_quat(q[[1,2,3,0]]).as_matrix()

def parse(path):
    windows={};factors={};alignments={};merges={};lines={};counts={}
    tokens=['KIMERA_CBS_REVISION_EDGE_ROW','FL4M_FROZEN_FACTOR_ROW',
            'KIMERA_CBS_MEAN_ALIGNMENT_ROW','KIMERA_CBS_POSE_MERGE_ROW']
    with pin(path).open() as f:
        for lineno,line in enumerate(f,1):
            for tag in tokens:
                if tag+',' not in line: continue
                fields=ANSI.sub('',line.split(tag+',',1)[1]).strip().split(',')
                k=int(fields[0]);counts[tag]=counts.get(tag,0)+1
                if tag==tokens[0]:
                    assert len(fields)==64
                    out=windows.setdefault(k,{})
                    for key,raw in [(int(fields[2]),fields[8:15]),(int(fields[3]),fields[15:22])]:
                        pp,rr=pose(raw)
                        if key in out:
                            assert np.array_equal(out[key][0],pp) and np.array_equal(out[key][1],rr)
                        out[key]=(pp,rr)
                    lines.setdefault(k,lineno)
                elif tag==tokens[1]: factors.setdefault(k,[]).append(fields)
                elif tag==tokens[2]: alignments.setdefault(k,[]).append(fields)
                else: merges.setdefault(k,[]).append(fields)
    return windows,factors,alignments,merges,lines,counts

def analyze(case,mode):
    name=case+'_'+mode
    d=load(H27/(case+'.json'))
    z=np.load(pin(H27/(case+'.npz')))
    record=d['old_input_products'][mode+'_kimera']
    path=pin(record['path'],record['sha256']);source=path.parent
    logfile=pin(source/'estimators.log',d['input_sha256'][str(source/'estimators.log')])
    windows,factors,alignments,merges,lines,counts=parse(logfile)
    t,ip,ir=a.load_curve(path)
    ap=z[mode+'_kimera_absolute_body_position'];ar=z[mode+'_kimera_absolute_body_rotation']
    assert np.array_equal(t,z[mode+'_kimera_timestamp_ns'])
    epoch=load(E/'all_method1_rerun_cpv33_20261029'/(case+'.json'))['epoch_ns']
    events=[];window_events=[];missing=[];max_abs_diff=0.;max_inc_t=0.;max_inc_r=0.
    for k in range(1,len(t)):
        if k not in windows or k not in windows[k] or k-1 not in windows[k]:
            missing.append(k);continue
        np_,nr=windows[k][k];rp,rr=windows[k][k-1]
        max_abs_diff=max(max_abs_diff,float(np.linalg.norm(np_-ap[k])))
        oldp,oldr=windows.get(k-1,{}).get(k-1,(ap[k-1],ar[k-1]))
        inc_t=ir[k-1].T@(ip[k]-ip[k-1]); inc_r=ir[k-1].T@ir[k]
        same_t=rr.T@(np_-rp);same_r=rr.T@nr
        max_inc_t=max(max_inc_t,float(np.linalg.norm(inc_t-same_t)))
        max_inc_r=max(max_inc_r,float(R.from_matrix(inc_r.T@same_r).magnitude()))
        # Body point, no global alignment: exact additive displacement decomposition.
        revision=rp-oldp;motion=np_-rp;absolute=np_-oldp
        assert np.max(abs(revision+motion-absolute))<1e-12
        # Common prism point gives the equivalent decomposition for the displayed trace.
        prism_old=oldp+oldr@a.TP;prism_prev=rp+rr@a.TP;prism_new=np_+nr@a.TP
        event=dict(k=k,time_s=float((t[k]-epoch)*1e-9),dt_s=float((t[k]-t[k-1])*1e-9),
          absolute_step_m=float(np.linalg.norm(absolute)),previous_revision_m=float(np.linalg.norm(revision)),
          same_update_motion_m=float(np.linalg.norm(motion)),
          previous_revision_rotation_deg=float(np.rad2deg(R.from_matrix(oldr.T@rr).magnitude())),
          prism_absolute_step_m=float(np.linalg.norm(prism_new-prism_old)),
          prism_previous_revision_m=float(np.linalg.norm(prism_prev-prism_old)),
          prism_same_update_motion_m=float(np.linalg.norm(prism_new-prism_prev)),
          absolute_step_xyz=absolute.tolist(),previous_revision_xyz=revision.tolist(),
          same_update_motion_xyz=motion.tolist(),saved_active_poses=len(windows[k]),
          new_factors=len(factors.get(k,[])),merges=len(merges.get(k,[])),log_line=lines[k])
        events.append(event)
        if k-1 not in windows: continue
        keys=sorted(set(windows[k])&set(windows[k-1]))
        if len(keys)<3: continue
        old=np.asarray([windows[k-1][i][0] for i in keys]);new=np.asarray([windows[k][i][0] for i in keys])
        rold=np.asarray([windows[k-1][i][1] for i in keys]);rnew=np.asarray([windows[k][i][1] for i in keys])
        # Best common orientation change; translation is least-squares for that rotation.
        delta_r=rnew@np.swapaxes(rold,1,2)
        rot=R.from_matrix(delta_r).mean().as_matrix()
        tr=np.mean(new-old@rot.T,axis=0)
        residual=new-(old@rot.T+tr)
        shifts=np.linalg.norm(new-old,axis=1)
        shape=np.linalg.norm(residual,axis=1)
        rot_res=np.rad2deg(R.from_matrix(np.swapaxes(delta_r,1,2)@rot).magnitude())
        centered_old=old-old.mean(0);centered_new=new-new.mean(0)
        denom=float(np.sum(centered_old**2))
        scale=float(np.sqrt(np.sum(centered_new**2)/denom)) if denom>1e-12 else None
        energy=float(np.sum((new-old)**2))
        event['window']=dict(overlap_poses=len(keys),min_id=min(keys),max_id=max(keys),
            old_extent_m=float(np.max(np.linalg.norm(centered_old,axis=1))*2),
            rigid_rotation_deg=float(np.rad2deg(R.from_matrix(rot).magnitude())),
            centroid_shift_m=float(np.linalg.norm(new.mean(0)-old.mean(0))),
            raw_shift_median_m=float(np.median(shifts)),raw_shift_max_m=float(shifts.max()),
            shape_residual_median_m=float(np.median(shape)),shape_residual_max_m=float(shape.max()),
            orientation_residual_max_deg=float(rot_res.max()),relative_extent_scale=scale,
            rigid_explained_fraction=1-float(np.sum(residual**2))/energy if energy>1e-20 else None)
        window_events.append(event)
    assert max_abs_diff<.001
    assert max_inc_t<1e-7 and max_inc_r<1e-7,(name,max_inc_t,max_inc_r)
    large=[e for e in window_events if e['previous_revision_m']>.5 and e['window']['old_extent_m']>.5]
    summary=dict(case=case,mode=mode,rows=len(t),source=str(source),counts=counts,
       paired_steps=len(events),missing_step_ids=missing,max_absolute_log_position_difference_m=max_abs_diff,
       same_update_vs_native_increment_max_translation_m=max_inc_t,
       same_update_vs_native_increment_max_rotation_rad=max_inc_r,
       absolute_step_m=stats([e['absolute_step_m'] for e in events]),
       previous_revision_m=stats([e['previous_revision_m'] for e in events]),
       same_update_motion_m=stats([e['same_update_motion_m'] for e in events]),
       revision_rotation_deg=stats([e['previous_revision_rotation_deg'] for e in events]),
       revisions_over_half_m=sum(e['previous_revision_m']>.5 for e in events),
       revisions_over_one_m=sum(e['previous_revision_m']>1 for e in events),
       large_moving_window_events=len(large),
       large_window_rigid_explained=stats([e['window']['rigid_explained_fraction'] for e in large]),
       large_window_shape_max_m=stats([e['window']['shape_residual_max_m'] for e in large]),
       large_window_orientation_residual_deg=stats([e['window']['orientation_residual_max_deg'] for e in large]),
       large_window_extent_scale=stats([e['window']['relative_extent_scale'] for e in large]),
       largest_events=sorted(events,key=lambda e:e['previous_revision_m'],reverse=True)[:12])
    save(H/(name+'_events.json'),events)
    save(H/(name+'_summary.json'),summary)
    print(json.dumps({k:v for k,v in summary.items() if k!='largest_events'},indent=2),flush=True)
    return summary

if __name__=='__main__':
    pin(__file__);pin(H/'PROTOCOL.txt')
    for mod in (m,m.c,m.c.b,a,a.g):pin(mod.__file__)
    summaries=[analyze(case,mode) for case,mode in [('velodyne_eig2','off'),('velodyne_eig2','on'),('hesai_eig2','on')]]
    vz=np.load(pin(H27/'velodyne_eig2.npz'));hz=np.load(pin(H27/'hesai_eig2.npz'))
    for suffix in ('timestamp_ns','absolute_body_position','absolute_body_rotation'):
        assert np.array_equal(vz['off_kimera_'+suffix],hz['off_kimera_'+suffix])
    assert all(sha(p)==s for p,s in PINS.items())
    save(H/'window_results.json',dict(passed=True,cases=summaries,hesai_velodyne_off_identical=True,
                                    input_sha256=PINS,inputs_unchanged=True))
