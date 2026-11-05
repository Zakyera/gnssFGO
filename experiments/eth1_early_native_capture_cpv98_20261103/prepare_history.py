"""Extract exact saved MAIN poses; no reconstruction or estimator execution."""
import csv,hashlib,json,re
from pathlib import Path
H=Path(__file__).resolve().parent;W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3');K=W/'runs'/H.name;P=K/'prepared';D=K/'historical_poses'
assert json.loads((H/'capture_complete.json').read_text())['passed'];D.mkdir()
sources={'old':W/'runs/eth1_split_imu_persistent_on_cpv91_20261102_attempt1/prepared/on_source',
 'new':W/'runs/eth1_split_imu_live_pair_cpv95_20261103_attempt3/prepared/full_on_source',
 'capture':P/'on_source'}
pins={};native={};summary={}
def sha(p):
    with p.open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
for label,S in sources.items():
    log=S/'estimators.log';pins[str(log)]=sha(log);poses={};stamps={};first={};cov={}
    with (S/'kimera_odometry.csv').open() as f:native[label]={int(r['field.header.seq']):int(r['field.header.stamp']) for r in csv.DictReader(f)}
    for line in log.open():
        if 'KIMERA_CBS_REVISION_EDGE_ROW,' not in line:continue
        a=line.split('KIMERA_CBS_REVISION_EDGE_ROW,')[1].strip().split(',');k=int(a[0])
        if k>25:break
        if k<3:continue
        assert len(a)==64
        pos=poses.setdefault(k,{})
        for idx,offset,stampcol in [(int(a[2]),8,4),(int(a[3]),15,5)]:
            value=tuple(map(float,a[offset:offset+7]));assert idx not in pos or pos[idx]==value;pos[idx]=value
            stamps[idx]=float(a[stampcol])
        if a[7]=='first' and (k-1,k)==tuple(map(int,a[2:4])) and k not in first:first[k]=list(map(float,a[28:]));cov[k]=float(a[5])
    for k in range(3,26):
        assert sorted(poses[k])==list(range(k+1)),(label,k,sorted(poses[k]))
        if label!='capture':
            with (D/f'{label}_{k}.txt').open('x') as f:
                for i,v in sorted(poses[k].items()):f.write(str(i)+' '+' '.join(format(x,'.17g') for x in v)+'\n')
    summary[label]=dict(first_covariance=first,to_stamps=cov)
for k in range(26):assert native['old'][k]==native['new'][k]==native['capture'][k],k
with (H/'historical_states.json').open('x') as f:json.dump(dict(passed=True,exact_native_key_timestamps=True,sources=summary,input_sha256=pins,omissions='Old complete visual measurements and velocity/bias histories are unavailable; historical pose-only contrasts are not full historical covariance reproductions.'),f,indent=2)
print('Historical poses extracted; all26 native key timestamps match CP91/95/capture')
