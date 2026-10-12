"""Archive small evidence and summarize the single shadow capture, not an ATE test."""
from pathlib import Path
import hashlib
import json
import shutil
import subprocess
import sys
import numpy as np
from prepare import HERE,RUN,ROOT,BUILD,OLD,sha
from oracle import Reader

def read(p):return json.loads(p.read_text())
assert read(RUN/'validation_logs/complete.json')['passed']
validation=read(ROOT/'profile40_on_validation.json');oracle=read(HERE/'real_oracle.json')
reader=Reader(ROOT/'profile40_on_source/native_gaussian.bin');sizes=[];last_counts=None;stage_count=0;commits=0
try:
    for tag,r in reader.records():
        if tag=='S':
            stage_count+=1
            sizes.append(dict(epoch=r['epoch'],native=len(r['refs']),keys=len(r['theta']),
              scalar_dimension=sum(v[0] for v in r['theta'].values()),retiring=len(r['retire']),fixed=len(r['fixed'])))
        if tag=='C':commits+=1;last_counts=r['counts']
finally:reader.f.close()
assert stage_count==commits==oracle['epochs']
sys.path.insert(0,str(HERE.parent/'shared_linearization_b0_20261012'))
from analyze_profile import analyze
baseline=analyze(OLD);candidate=analyze(RUN)
q=oracle['queries'];maximums={k:max(r[k] for r in q) for k in q[0] if k.endswith('_relative') or k=='condition'}
query_seconds=[r['query_seconds'] for r in q]
def stats(a):return dict(mean=float(np.mean(a)),median=float(np.median(a)),p95=float(np.quantile(a,.95)),max=float(max(a)),total=float(sum(a)))
report=dict(passed=True,epochs=commits,retirement_events=oracle['retirements'],
  counts=dict(zip(['admitted','active','absorbed','removed'],last_counts)),
  max_native_active_before_retirement=max(r['native'] for r in sizes),max_values=max(r['keys'] for r in sizes),
  max_dimension=max(r['scalar_dimension'] for r in sizes),
  query_count=len(q),query_epochs=[r['epoch'] for r in q],oracle_maxima=maximums,
  recorder_stage_seconds=oracle['io_seconds'],query_seconds=stats(query_seconds),
  journal_bytes=(ROOT/'profile40_on_source/native_gaussian.bin').stat().st_size,
  source_coverage=read(ROOT/'profile40_on_source/source_coverage.json'),
  receiver=dict(versions=validation['versions'],active_source_ids=validation['active_source_ids'],
    replacements=validation['replacements'],unsupported=len(validation['unsupported']),
    max_gradient=validation['max_publication_gradient'],age=validation['live_output_age_s'],
    queue=validation['final_transport_status']['max_queue'],repeat_files=validation['repeat_files']),
  controller=read(ROOT/'profile40_on_control/complete.json'),baseline_timing=baseline,shadow_timing=candidate,
  caveat='Dual covariance paths and recording enabled; not candidate-only speed or independent LOCAL covariance equivalence.')
(HERE/'summary.json').write_text(json.dumps(report,indent=2))
out=HERE/'evidence';out.mkdir(exist_ok=False)
def copy(src,dest):
    dst=out/dest;dst.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(src,dst)
for name in ['profile40_on_validation.json','profile40_on_native_validation.json','profile40_on_lifecycle_validation.json',
             'profile40_on_diagnostics_validation.json','profile40_on.json','protected_manifest.json',
             'profile40_on_source/effective_mode_check.json','profile40_on_source/source_coverage.json',
             'profile40_on_control/complete.json','profile40_on_receiver/transport_status.json','preflight/validation.json']:
    copy(ROOT/name,'capture/'+name)
for name in ['ready.json','result.json','map_guard.log','command.txt','params.json']:copy(RUN/'no_data_startup'/name,'startup/'+name)
for label,path in [('release',BUILD),('asan',BUILD.parent/'asan_1')]:
    for name in ['build_result.json','commands.json','tests/result.json','tests/tests.xml','tests/tests.log']:
        copy(path/name,label+'/'+name)
for name in ['result.json','regression.xml']:copy(BUILD/'nonregression'/name,'nonregression/'+name)
for name in ['experiment.py','run_case.py','source.env','source_runner.sh','verify_effective.py','map_guard.py','glim_profile.launch','receiver_e2.sh']:
    copy(ROOT/name,'harness/'+name)
for name in ['ShadowObserver.cpp','ShadowObserver.h','Recording.h','test_lifecycle.cpp']:
    copy(BUILD/'source'/name,'private_source/'+name)
for name in ['incremental_fixed_lag_smoother_ext.cpp','incremental_fixed_lag_smoother_ext_with_fallback.cpp']:
    copy(BUILD/'overlay'/name,'private_source/'+name)
copy(RUN/'validation_logs/complete.json','capture/validation_complete.json')
(out/'hashes.json').write_text(json.dumps({str(p.relative_to(out)):sha(p) for p in sorted(out.rglob('*')) if p.is_file()},indent=2))
print(json.dumps({k:v for k,v in report.items() if k not in ['baseline_timing','shadow_timing','receiver','source_coverage']},indent=2))
