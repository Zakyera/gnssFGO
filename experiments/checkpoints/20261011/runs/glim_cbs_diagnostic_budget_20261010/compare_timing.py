"""Read saved timing rows; compare identical scan stamps, never score accuracy."""
from pathlib import Path
import hashlib
import json
import re
import numpy as np

ROOT=Path(__file__).resolve().parent
OLD=ROOT.parent/'gnssfgo_eth1_covariance160_20261010/prepared'
T0=1727775008.003475
COLS='stamp frame points nimu nfactors active marginalized lookup inter_imu imufactor intra_imu deskew pointcov cpuframe createframe createfactors pre_callback smoother post_callback marginalize updateframes imuvalidate outputcallbacks total'.split()
ANSI=re.compile(r'\x1b\[[0-9;]*m')
ROS=re.compile(r'\[([0-9.]+), ([0-9.]+)\]')

def stats(a):
    a=np.array(a,dtype=float)
    if not len(a):return dict(n=0)
    return dict(n=len(a),mean=float(a.mean()),median=float(np.median(a)),
                p95=float(np.quantile(a,.95)),max=float(a.max()))

def read(source):
    path,=[p for p in (source/'ros_logs').glob('*/glim_rosnode-*-stdout.log') if p.parent.name!='latest']
    scan={}; cbs={}; diag=[]; active_rows=0; warnings=0; inputs=[]; errors=[]
    competition=set()
    h=hashlib.sha256()
    for raw in path.open():
        h.update(raw.encode());line=ANSI.sub('',raw)
        if 'MEAN_ALIGNMENT_UNAVAILABLE' in line:errors.append(line.strip())
        if 'sync mode seriously affects' in line:warnings+=1
        if 'GLIM_CBS_ACTIVE_FACTOR_DIAGNOSTIC_ROW,' in line:active_rows+=1
        if 'CBS_RECEIVER_LOCAL_COMPETITION_ROW,K2G,' in line:
            competition.add(line.split('CBS_RECEIVER_LOCAL_COMPETITION_ROW,')[1].split(',')[7])
        if 'GLIM_CBS_DIAGNOSTIC_TIMING_ROW,' in line:
            diag.append(list(map(float,line.split('GLIM_CBS_DIAGNOSTIC_TIMING_ROW,')[1].split(','))))
        if 'GLIM_ODOM_IMU_TIMING_ROW,' in line:
            a=line.split('GLIM_ODOM_IMU_TIMING_ROW,')[1].strip().split(',')
            assert len(a)==25 and a[-1]=='processed'
            r=dict(zip(COLS,map(float,a[:-1])));scan[int(r['frame'])]=r
        if 'GLIM_CBS_TIMING_ROW,' in line:
            a=line.split('GLIM_CBS_TIMING_ROW,')[1].strip().split(',')
            ts=ROS.search(line)
            if ts:cbs.setdefault(a[0],{})[int(a[1])]=(list(map(float,a[1:])),float(ts[2]))
        if 'GLIM_ROS_INPUT_TIMING_ROW,' in line:
            inputs.append(list(map(float,line.split('GLIM_ROS_INPUT_TIMING_ROW,')[1].split(','))))
    stderr_warnings=sum('sync mode seriously affects' in line for line in (source/'estimators.log').open())
    return dict(scan=scan,cbs=cbs,summary=dict(path=str(path),sha256=h.hexdigest(),scans=len(scan),
        diagnostic_calls=len(diag),max_diagnostic_samples=max([r[2] for r in diag],default=None),
        diagnostic_ms=stats([r[1] for r in diag]),active_diagnostic_rows=active_rows,
        gpu_sync_warnings_in_glim_stdout=warnings,
        gpu_sync_warnings_in_estimator_stderr=stderr_warnings,competition_samples=len(competition),
        input_backpressure_ms=stats([r[5] for r in inputs]),max_input_workload=max(r[8] for r in inputs),
        native_alignment_failures=errors))

def main():
    sources=dict(old_off=OLD/'full160_off_source',old_on=OLD/'full160_on_source',
                 new_on=ROOT/'prepared/perf100_on_source')
    data={k:read(v) for k,v in sources.items()}
    result=dict(cases={k:d['summary'] for k,d in data.items()},matched_windows=[])
    for lo,hi in [(0,95),(0,20),(20,40),(40,60),(60,80),(80,95)]:
        ids=[i for i,r in data['new_on']['scan'].items() if lo<=r['stamp']-T0<hi]
        assert ids
        stamps=[data['new_on']['scan'][i]['stamp'] for i in ids]
        window=dict(seconds=[lo,hi],scans=len(ids),cases={})
        for name,d in data.items():
            assert [d['scan'][i]['stamp'] for i in ids]==stamps
            stats_scan={k:stats([d['scan'][i][k] for i in ids]) for k in ['pre_callback','smoother','post_callback','total']}
            nested={}
            for kind,columns in {
              'on_smoother_update':{'sidecar_update':10},
              'publish_outgoing':{'local_covariance_rebuild':17,'belief_query':18,'ros_publish':20},
              'on_smoother_update_finish':{'outgoing_plus_revisions':12}}.items():
                rows=[d['cbs'][kind][i][0] for i in ids if i in d['cbs'].get(kind,{})]
                nested.update({k:stats([r[j] for r in rows]) for k,j in columns.items()})
            pub=d['cbs'].get('publish_odometry',{})
            age=stats([pub[i][1]-d['scan'][i]['stamp'] for i in ids if i in pub])
            window['cases'][name]=dict(scan_ms=stats_scan,nested_ms=nested,publication_age_s=age)
        result['matched_windows'].append(window)
    assert data['new_on']['summary']['active_diagnostic_rows']==0
    assert data['new_on']['summary']['max_diagnostic_samples']==0
    with (ROOT/'timing_comparison_verified.json').open('x') as f:json.dump(result,f,indent=2)
    print('MATCHED TIMING COMPARISON SAVED; warnings counted separately in estimator stderr')

if __name__=='__main__':main()
