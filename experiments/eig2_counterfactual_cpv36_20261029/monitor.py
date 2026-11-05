"""Compact read-only capture/parity check. Does not launch or modify a run."""
import csv,json
from pathlib import Path
import numpy as np
H=Path(__file__).resolve().parent;W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
P=W/'runs/eig2_counterfactual_cpv36_20261029';C=P/'capture';S=W/'runs/gnssfgo_eig2_velodyne_full_cpv4_20261024/prepared/full_off_source'
def costs(p):
    rows=[]
    for l in p.read_text().splitlines():
        a=l.split(',')
        if len(a)==10:
            try:rows.append([float(x) for x in a])
            except ValueError:pass
    return np.array(rows)
a=costs(C/'objective.csv');b=costs(S/'objective.csv')[:len(a)]
nondata_lines={}
def poses(p):
    records=[];warnings=[]
    allowed={'WARNING: no messages received and simulated time is active.','Is /clock being published?'}
    for lineno,d in enumerate(csv.DictReader(p.open()),2):
        if d.get('%time') in allowed and all(v is None for k,v in d.items() if k!='%time'):
            warnings.append({'line':lineno,'text':d['%time']});continue
        assert None not in d and all(v is not None for v in d.values()) and (d.get('field.header.seq') or '').isdigit(),(p,lineno,'malformed data row')
        records.append(d)
    nondata_lines[str(p)]=warnings
    return records
x=poses(C/'odometry.log');y=poses(S/'kimera_odometry.csv');diff=[]
for i,d in enumerate(x):
    for k,v in d.items():
        if k=='%time':continue
        ov=y[i][k]
        try:equal=(float(v)==float(ov)) if 'stamp' not in k and 'seq' not in k else int(v)==int(ov)
        except ValueError:equal=v==ov
        if not equal:diff.append((i,k,v,ov))
status=json.loads((C/'status.json').read_text()) if (C/'status.json').exists() else {}
result=dict(status=status,native_rows=len(x),objective_rows=len(a),objective_exact=np.array_equal(a,b),native_exact=len(diff)==0,first_native_mismatches=diff[:2],max_cost_abs_diff=float(np.abs(a-b).max()) if len(a) else None,last_sensor_s=(int(x[-1]['field.header.stamp'])-1730638816563546034)/1e9 if x else None,capture_complete=(C/'capture_complete.json').exists(),failed=(C/'failed.txt').read_text() if (C/'failed.txt').exists() else None)
result['nondata_lines']=nondata_lines
print(json.dumps(result,indent=2))

