"""Supplementary read-only source/receiver counters from the same saved run."""
from pathlib import Path
import csv
import hashlib
import json
import numpy as np

ROOT = Path(__file__).resolve().parent
P = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_eth1_vw_jacobian_live160_20261011/prepared')
S = P/'full160_on_source'
R = P/'full160_on_receiver/receiver'
read = lambda p: list(csv.DictReader(p.open()))
def stats(a):
    a=np.asarray(a,dtype=float)
    if not len(a): return {'n':0}
    return dict(n=len(a), mean=float(a.mean()), median=float(np.median(a)),
                p95=float(np.quantile(a,.95)), max=float(a.max()))
conv=read(R/'publication_convergence.csv')
steps=read(R/'publication_steps.csv')
recovery_ids={r['index'] for r in steps if r['recovery']=='1'}
ratios=[int(r['relinearized'])/(3*(int(r['index'])+1)) for r in steps if r['recovery']=='0']
ktimes={int(r['field.header.seq']):(int(r['field.header.stamp'])-1727775008003475000)*1e-9
        for r in read(S/'kimera_odometry.csv')}
krows={}
markers=['KIMERA_BACKEND_SPINONCE_TIMING_ROW,','KIMERA_OPTIMIZE_TIMING_ROW,','KIMERA_CBS_OUTGOING_TIMING_ROW,']
warning_bytes=0
for line in (S/'estimators.log').open():
    if 'sync mode seriously affects' in line: warning_bytes+=len(line.encode())
    for marker in markers:
        if marker in line:
            row=line.split(marker)[1].strip().split(',')
            krows.setdefault(marker,{})[int(row[0])]=list(map(float,row[1:]))
            break
glog,=[p for p in (S/'ros_logs').glob('*/glim_rosnode-*-stdout.log') if p.parent.name!='latest']
gpu=[]
for line in glog.open():
    marker='GLIM_GPU_TIMING_ROW,create_factors,'
    if marker in line:
        a=line.split(marker)[1].strip().split(',')
        gpu.append((float(a[0])-1727775008.003475, int(a[3])))
windows=[]
for lo,hi in [(0,20),(20,40),(40,60),(60,80),(80,100),(100,120),(120,130),(130,144),(0,144)]:
    k={marker:stats([r[0]/1000 for i,r in rows.items() if lo<=ktimes.get(i,-1)<hi])
       for marker,rows in krows.items()}
    windows.append(dict(seconds=[lo,hi],kimera_wall_seconds=k,
        glim_keyframe_count=stats([count for t,count in gpu if lo<=t<hi])))
def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        for block in iter(lambda:f.read(1024*1024),b''): h.update(block)
    return h.hexdigest()
manifest=json.loads((P/'protected_manifest.json').read_text())
mismatches=[p for p,h in manifest.items() if sha(p)!=h]
result=dict(relinearized_fraction=stats(ratios),
    ordinary_refinement_step_count=len(ratios), full_relinearization_steps=sum(r==1 for r in ratios),
    recovery_publications=len(recovery_ids),
    recovery_publication_seconds=sum(float(r['seconds']) for r in conv if r['index'] in recovery_ids),
    other_publication_seconds=sum(float(r['seconds']) for r in conv if r['index'] not in recovery_ids),
    warning_bytes=warning_bytes, total_estimator_log_bytes=(S/'estimators.log').stat().st_size,
    windows=windows, protected_files_checked=len(manifest), protected_mismatches=mismatches)
with (ROOT/'saved_supplement.json').open('x') as f: json.dump(result,f,indent=2)
print(json.dumps({k:v for k,v in result.items() if k!='windows'},indent=2))
for w in windows: print(w)
assert not mismatches
