"""Associate existing passive competition diagnostics with scan callbacks."""
from pathlib import Path
import hashlib,json,math,re,statistics

H=Path(__file__).resolve().parent
base=json.loads((H/'timing_v2.json').read_text())
ansi=re.compile(r'\x1b\[[0-9;]*m')
def stats(values):
    values=sorted(values)
    return dict(n=len(values),median=statistics.median(values),
                p95=values[math.ceil(.95*len(values))-1],max=values[-1]) if values else {'n':0}
out={}
for name,case in base['cases'].items():
    if case['mode']!='on':continue
    root=Path(case['source'])/'full_on_source'
    paths={p.resolve() for p in root.glob('ros_logs/*/glim_rosnode-*-stdout.log')}
    assert len(paths)==1
    path=paths.pop()
    with path.open('rb') as f:assert hashlib.file_digest(f,'sha256').hexdigest()==base['inputs'][str(path)]['sha256']
    pending=[];rows=[];timestamps={}
    for raw in path.open():
        line=ansi.sub('',raw)
        marker='CBS_RECEIVER_LOCAL_COMPETITION_ROW,'
        failure='CBS_RECEIVER_LOCAL_COMPETITION_STATUS_ROW,'
        if marker in line or failure in line:
            prefix=marker if marker in line else failure
            f=line.split(prefix,1)[1].strip().split(',')
            pending.append(dict(status='ok' if prefix==marker else 'unavailable',sample=f[7],seen=f[8]))
        elif 'GLIM_CBS_TIMING_ROW,on_smoother_update,' in line:
            f=line.split('GLIM_CBS_TIMING_ROW,on_smoother_update,',1)[1].strip().split(',')
            rows.append(dict(id=int(f[0]),inject_loop_ms=float(f[9]),diagnostics=pending))
            pending=[]
        elif 'GLIM_ODOM_IMU_TIMING_ROW,' in line:
            f=line.split('GLIM_ODOM_IMU_TIMING_ROW,',1)[1].strip().split(',')
            timestamps[int(f[1])]=float(f[0])
    assert not pending
    for r in rows:r['stamp']=timestamps.get(r['id'])
    tagged=[r for r in rows if r['diagnostics']]
    untagged=[r for r in rows if not r['diagnostics']]
    over=[r for r in rows if r['inject_loop_ms']>200]
    out[name]=dict(diagnostic_records=sum(len(r['diagnostics']) for r in tagged),
        status_counts={s:sum(d['status']==s for r in tagged for d in r['diagnostics']) for s in ['ok','unavailable']},
        tagged_inject_ms=stats([r['inject_loop_ms'] for r in tagged]),
        untagged_inject_ms=stats([r['inject_loop_ms'] for r in untagged]),
        above_200ms=len(over),above_200ms_tagged=sum(bool(r['diagnostics']) for r in over),
        top_callbacks=sorted(rows,key=lambda r:r['inject_loop_ms'],reverse=True)[:8])
    print(name,json.dumps({k:v for k,v in out[name].items() if k!='top_callbacks'}))
with (H/'audit_spikes.json').open('x') as f:json.dump(out,f,indent=2)
