"""Read-only integrity/status checks; writes new CP-K6 evidence only."""
import hashlib
import json
from pathlib import Path
import subprocess

H = Path(__file__).resolve().parent
R = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/kimera_prior_refresh_cpk6_20261019')

def digest(p):
    h = hashlib.sha256()
    with open(p,'rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''):
            h.update(b)
    return h.hexdigest()

protected = json.loads((R/'protected_before.json').read_text())
bad = [p for p,h in protected.items() if not Path(p).is_file() or digest(p)!=h]
inputs = json.loads((R/'inputs.json').read_text())
changed_inputs = [p for p,h in inputs.items() if digest(p)!=h]
# Exactly one compiler-harness correction: select bundled Eigen before system Eigen.
assert changed_inputs == [str(R/'build.py')], changed_inputs
assert (H/'test.cpp').read_bytes() == (R/'test.cpp').read_bytes()
assert (H/'build.py').read_bytes() == (R/'build.py').read_bytes()
for name in ('release_a2','san_a1'):
    assert json.loads((R/name/'passed.json').read_text())['passed']
assert (R/'release_a2/run0.log').read_bytes()==(R/'release_a2/run1.log').read_bytes()
status = json.loads(subprocess.check_output(['docker','inspect',
    'cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'],text=True))
containers = {c['Name']:c['State']['Status'] for c in status}
assert set(containers.values())=={'exited'}, containers
metrics = {}
for line in (R/'release_a2/run0.log').read_text().splitlines():
    if '=' in line:
        k,v=line.split('=',1); metrics[k]=float(v)
evidence = {
    'passed':not bad,'protected_count':len(protected),'changed_protected_files':bad,
    'containers':containers,'repeat_byte_identical':True,
    'asan_ubsan_passed':True,'vendor_dsos_instrumented':False,
    'initial_compile_only_failure':'release_a1: missing GTSAM bundled Eigen include path; no test executed',
    'test_inputs_unchanged':True,'build_harness_change':changed_inputs,
    'final_test_source_sha256':digest(R/'test.cpp'),
    'final_build_script_sha256':digest(R/'build.py'),
    'reference_prior_helper_sha256':digest(R/'OriginalChartPrior.h'),
    'metrics':metrics,'scope':'synthetic coordinate/information gate, NOT live integration or convergence',
}
with (H/'validation.json').open('x') as f:
    json.dump(evidence,f,indent=2); f.write('\n')
assert not bad,bad
print(json.dumps({'passed':True,'protected':len(protected),'containers':containers,'metrics':metrics}))
