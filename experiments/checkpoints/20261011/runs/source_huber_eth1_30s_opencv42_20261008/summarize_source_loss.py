"""Summarize diagnostic records only; never modifies estimators or trajectories."""
from pathlib import Path
import collections
import json
import re
import numpy as np

ROOT=Path(__file__).resolve().parent

def stats(values):
    a=np.asarray(values,dtype=float)
    if not len(a): return dict(count=0)
    assert np.isfinite(a).all()
    return dict(count=len(a),minimum=float(a.min()),median=float(np.median(a)),
                mean=float(a.mean()),p95=float(np.quantile(a,.95)),maximum=float(a.max()))

def analyze(path):
    creations={}; observations=collections.defaultdict(list)
    snapshots=[]; current=None
    for line in path.open(errors='strict'):
        if 'CBS_SOURCE_LOSS_' not in line: continue
        row=line.split('CBS_SOURCE_LOSS_',1)[1].strip().split(',')
        if row[0]=='CREATE':
            identity,serial=row[1:3]
            assert serial not in creations
            covariance=np.array(list(map(float,row[6:]))).reshape(6,6)
            creations[serial]=dict(identity=identity,keys=row[3:5],loss=row[5],
                rot_trace=float(np.trace(covariance[:3,:3])),rho_trace=float(np.trace(covariance[3:,3:])),
                eigen_min=float(np.linalg.eigvalsh((covariance+covariance.T)/2).min()))
        elif row[0]=='STATE':
            identity,serial,phase=row[1:4]
            observations[phase].append(dict(identity=identity,serial=serial,keys=row[4:6],
                s=float(row[6]),weight=float(row[7]),cost=float(row[8]),
                rotation_norm=float(row[9]),rho_norm=float(row[10])))
            if phase=='post_fresh':
                assert current is not None
                current.add(serial)
        elif row[0]=='SNAPSHOT_BEGIN':
            assert current is None
            current=set()
        elif row[0]=='SNAPSHOT_END':
            assert current is not None
            snapshots.append(current); current=None
    assert current is None
    for rows in observations.values():
        assert all(r['serial'] in creations for r in rows)
    phases={}
    for phase,rows in observations.items():
        phases[phase]={name:stats([r[name] for r in rows]) for name in ['s','weight','cost','rotation_norm','rho_norm']}
        phases[phase]['fraction_weight_lt_1']=float(np.mean([r['weight']<1 for r in rows]))
        phases[phase]['fraction_weight_lt_0_1']=float(np.mean([r['weight']<.1 for r in rows]))
    by_id=collections.Counter(r['identity'] for r in creations.values())
    return dict(created_instances=len(creations),unique_source_identities=len(by_id),
        instances_per_identity=stats(list(by_id.values())),
        full_covariance={k:stats([c[k] for c in creations.values()]) for k in ['rot_trace','rho_trace','eigen_min']},
        phases=phases,snapshots=len(snapshots),active_instances=stats([len(s) for s in snapshots]),
        instances_exiting_consecutive_snapshots=sum(len(a-b) for a,b in zip(snapshots,snapshots[1:])),
        final_active_instances=len(snapshots[-1]) if snapshots else None,
        limitations='Linearization hooks include covariance/audit calls, not only optimizer calls. Snapshot exits are not individually attributed to replacement vs marginalization. No source stationarity guarantee.')

def main():
    result={}
    for case in ['off_gaussian','on_gaussian','on_huber']:
        validation_path=ROOT/(case+'_validation.json')
        if validation_path.exists():
            validation=json.loads(validation_path.read_text())
            assert validation['passed']
        else:
            assert case=='on_huber'
            blocked=json.loads((ROOT/'repeat_difference.json').read_text())
            assert blocked['status']=='VALIDATION_BLOCKED'
            validation=dict(passed=False,max_publication_gradient=blocked['live_stationarity']['maximum_gradient'],
                            live_output_age_s=None)
        source=ROOT/(case+'_source')
        result[case]={}
        for kind,glob in [('glim','*/glim_rosnode-*-stdout.log'),('kimera','*/kimera_vio_ros-kimera_vio_ros_node-*-stdout.log')]:
            paths=sorted({p.resolve() for p in (source/'ros_logs').glob(glob)})
            assert len(paths)==1
            result[case][kind]=analyze(paths[0])
        result[case]['capture_seconds']=json.loads((ROOT/(case+'_control/complete.json')).read_text())['seconds']
        result[case]['receiver_max_publication_gradient']=validation['max_publication_gradient']
        result[case]['receiver_output_age_s']=validation['live_output_age_s']
        result[case]['full_validation_passed']=validation['passed']
    with (ROOT/'source_loss_summary.json').open('x') as f: json.dump(result,f,indent=2)
    print(json.dumps(result,indent=2))

if __name__=='__main__': main()
