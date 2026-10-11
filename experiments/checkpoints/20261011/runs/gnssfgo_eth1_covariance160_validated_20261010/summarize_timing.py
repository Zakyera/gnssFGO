"""Reuse the diagnosed timing parser; no optimization or estimator changes."""
from pathlib import Path
import importlib.util
import json
ROOT=Path(__file__).resolve().parent
PERF=ROOT.parent/'glim_cbs_diagnostic_budget_20261010'
OLD=ROOT.parent/'gnssfgo_eth1_covariance160_20261010/prepared'
spec=importlib.util.spec_from_file_location('retained_timing',PERF/'compare_timing.py')
t=importlib.util.module_from_spec(spec);spec.loader.exec_module(t)
assert json.loads((ROOT/'prepared/full160_on_validation.json').read_text())['passed']
data={name:t.read(path) for name,path in {
    'reused_off':OLD/'full160_off_source',
    'corrected_on':ROOT/'prepared/full160_on_source'}.items()}
on=data['corrected_on']
assert on['summary']['max_diagnostic_samples']==0
assert on['summary']['active_diagnostic_rows']==0
assert not on['summary']['native_alignment_failures']
assert len(on['scan'])>=1580
windows=[]
for lo,hi in [(0,160),*[(i,i+20) for i in range(0,160,20)]]:
    ids=[i for i,r in on['scan'].items() if lo<=r['stamp']-t.T0<hi]
    stamps=[on['scan'][i]['stamp'] for i in ids]
    w=dict(seconds=[lo,hi],matched_scans=len(ids),cases={})
    for name,d in data.items():
        assert [d['scan'][i]['stamp'] for i in ids]==stamps
        pub=d['cbs']['publish_odometry']
        w['cases'][name]=dict(scan_ms={k:t.stats([d['scan'][i][k] for i in ids])
            for k in ['pre_callback','smoother','post_callback','total']},
            publication_age_s=t.stats([pub[i][1]-d['scan'][i]['stamp'] for i in ids if i in pub]))
    windows.append(w)
result=dict(passed=True,cases={k:v['summary'] for k,v in data.items()},windows=windows,
    caveat='Same physical scan timestamps; different CBS modes and asynchronous arrival histories. No source bit-equivalence claim.')
with (ROOT/'timing_summary.json').open('x') as f:json.dump(result,f,indent=2)
print(json.dumps(windows[0],indent=2))
