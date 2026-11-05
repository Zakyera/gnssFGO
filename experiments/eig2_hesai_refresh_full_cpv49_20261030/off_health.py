"""Saved OFF admission: direct EVO native Method1, revisions; no estimator calls."""
import importlib.util,json,sys
from pathlib import Path
import numpy as np
sys.dont_write_bytecode=True
H=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('report',H/'evaluate.py');r=importlib.util.module_from_spec(spec);spec.loader.exec_module(r)
P=r.P;S=P/'full_off_source';O=H/'off_health_evaluation'
assert not O.exists() and not (H/'off_health.json').exists()
assert json.loads((H/'off_validated.json').read_text())['passed']
record=next(x for x in r.m.c.b.ROWS if x[0]=='hesai_eig2')
_,ev,refobj=r.m.c.b.load(record)
reference=(refobj.times,np.asarray(refobj.poses)[:,:3,3],np.asarray(refobj.poses)[:,:3,:3])
absolute,_,checks=r.m.kimera(S)
native=r.a.load_curve(S/'kimera_odometry.csv')
revision,events=r.d39.window_summary(S/'estimators.log',absolute[0],native,absolute)
ts,p,rot=absolute;ep,er=r.a.prism(p,rot)
gp,gr,valid=r.a.sample(reference,ts,50_000_000);assert valid.all()
gp,gr=r.a.prism(gp,gr)
segments=np.split(np.arange(len(ts)),np.flatnonzero(np.diff(ts)>500_000_000)+1)
O.mkdir();rp=O/'reference.tum';ref=r.export(rp,r.curve(ts,gp,gr))
refs,pairs,info=r.reference_segments(O,ts,gp,gr,segments)
metrics=r.score_series(O,'kimera_off',ts,ep,er,gp,gr,segments,refs,rp,ref)
result=dict(passed=revision['revisions_over_half_m']==0,
 criterion='Zero previous-state position revisions >0.5m, as in corrected CP-V42 OFF; not universal health proof.',
 method='Method1 newest absolute backend; native cadence with gaps excluded',
 metrics=metrics,revision=revision,reconstruction=checks,evo_checks=r.checks,
 commands=r.commands,source_runs=0,receiver_runs=0,reference='IE-TC shares CPT7; not independent prism GT')
r.save(H/'off_health.json',result);r.save(O/'revision_events.json',events)
print(json.dumps(result,indent=2))
assert result['passed'],'OFF large-revision screen failed; ON not admitted; no retries'
