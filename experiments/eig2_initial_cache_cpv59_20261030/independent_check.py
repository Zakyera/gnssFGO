"""Independent SciPy check of reported CP59 pose/bias differences; no solves."""
import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

H=Path(__file__).resolve().parent
P=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs')/H.name/'attempt_exact_values/results'
a=json.loads((H/'analysis.json').read_text())
states={n:json.loads((P/(n+'.json')).read_text()) for n in ['previous','A_ordinary','B_ordinary','A_refresh','B_refresh']}
checks=0;max_error=0.0
def compare(first,second,expected,keys):
    global checks,max_error
    for k in keys:
        n=k[1:];p=first[k];q=second[k]
        rp=Rotation.from_quat([p[4],p[5],p[6],p[3]])
        rq=Rotation.from_quat([q[4],q[5],q[6],q[3]])
        measured=dict(position_m=np.linalg.norm(np.subtract(q[:3],p[:3])),rotation_deg=np.degrees((rq*rp.inv()).magnitude()),
                      velocity_m_s=np.linalg.norm(np.subtract(second['v'+n],first['v'+n])),
                      accel_bias_m_s2=np.linalg.norm(np.subtract(second['b'+n][:3],first['b'+n][:3])),
                      gyro_bias_rad_s=np.linalg.norm(np.subtract(second['b'+n][3:],first['b'+n][3:])))
        for field,value in measured.items():
            error=abs(value-expected['per_state'][k][field]);max_error=max(error,max_error)
            assert error<1e-10,(k,field,error);checks+=1
    for field in measured:
        assert expected['maxima'][field]==max(row[field] for row in expected['per_state'].values())

old=[f'x{i}' for i in range(6)]
for arm in ['A','B']:
    compare(states['previous'],states[arm+'_ordinary'],a['arms'][arm]['ordinary'],old)
    compare(states[arm+'_ordinary'],states[arm+'_refresh'],a['arms'][arm]['refresh'],old)
    compare(states['previous'],states[arm+'_refresh'],a['arms'][arm]['net'],old)
for phase in ['ordinary','refresh']:compare(states['A_'+phase],states['B_'+phase],a['A_to_B'][phase],[f'x{i}' for i in range(7)])
classification={}
for phase in ['ordinary','refresh']:
    rows={arm:{r['slot']:r['status'] for line in (P/(arm+'_'+phase+'_factors.jsonl')).open()
               for r in [json.loads(line)] if r['type'].startswith('smart')} for arm in ['A','B']}
    assert rows['A'].keys()==rows['B'].keys()
    classification[phase]=[slot for slot in rows['A'] if rows['A'][slot]!=rows['B'][slot]]
inputs=[H/'analysis.json',Path(__file__)]+[p for p in P.iterdir() if p.is_file()]
record=dict(passed=True,scalar_metric_checks=checks,max_absolute_metric_difference=max_error,
            original_rule_postevaluation_classification_differences=classification,
            input_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs})
with (H/'independent_check.json').open('x') as f:json.dump(record,f,indent=2)
print(json.dumps({k:v for k,v in record.items() if k!='input_sha256'},indent=2))
