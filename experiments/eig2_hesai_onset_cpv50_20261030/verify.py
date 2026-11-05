"""Independent arithmetic and nonmutation checks; no source/solver execution."""
import datetime
import json
import subprocess
import sys
from pathlib import Path
import numpy as np
from scipy.linalg import expm
sys.dont_write_bytecode = True
import analyze as a

assert not (a.H/'verification.json').exists()
inputs = {}
data = {}
for name in ('analysis', 'transport_means', 'chronology'):
    data[name] = json.loads((a.H/f'{name}.json').read_text())
    for p, want in data[name]['input_sha256'].items():
        assert p not in inputs or inputs[p] == want
        inputs[p] = want
for p,want in inputs.items():
    assert a.sha(p) == want, p
old = json.loads((a.H49/'complete.json').read_text())['output_sha256']
protected = json.loads((a.P/'protected_manifest.json').read_text())
report_inputs = json.loads((a.H49/'evaluation/input_sha256.json').read_text())
for mapping in (old, protected, report_inputs):
    for p,want in mapping.items():
        assert a.sha(p) == want, p
checkpoint = a.E/'checkpoints/20261023_hesai_four_sequences'
manifest = json.loads((checkpoint/'manifest.json').read_text())
for name,record in manifest['files'].items():
    for p in (checkpoint/name, Path(record['original'])):
        assert p.stat().st_size == record['bytes'] and a.sha(p) == record['sha256'], p

rng = np.random.default_rng(50)
maximum = 0.
composition_maximum = 0.
for _ in range(100):
    x = rng.normal(size=6); x[:3] *= .2
    hat = np.zeros((4,4)); hat[:3,:3] = a.skew(x[:3]); hat[:3,3] = x[3:]
    maximum = max(maximum,float(np.max(abs(a.exp(x)-expm(hat)))))
    assert np.max(abs(a.log(a.exp(x))-x)) < 1e-10
    A,S,T,B,G = [a.exp(rng.normal(size=6)) for _ in range(5)]
    # Endpoint-time bridge: A->S, S->T, T->B equals A->B.
    aligned = (a.inv(A)@S)@(a.inv(S)@T)@(a.inv(T)@B)
    composition_maximum = max(composition_maximum,float(np.max(abs(aligned-a.inv(A)@B))))
    # A shared world-frame change cannot alter a body-frame relative pose.
    assert np.max(abs(a.inv(G@S)@(G@T)-a.inv(S)@T)) < 1e-10
assert max(maximum, composition_maximum) < 1e-10

main = data['analysis']; means = data['transport_means']; chronology = data['chronology']
edges = json.loads((a.H/'edge_events.json').read_text())
assert len(edges) == len({(e['k'],*e['receiver']) for e in edges}) == 1743
assert main['missing_geometry'] == []
assert main['modes']['on']['first_marginalization']['k'] == 26
assert main['modes']['on']['first_peer_retirement']['k'] == 30
assert main['modes']['off']['first_peer_retirement'] is None
assert main['modes']['on']['recovery_events'] == []
assert main['modes']['on']['total_cost_increases'] == []
assert all(e['same_pose_values'] for e in chronology['early_pose_comparisons'] if e['k'] <= 5)
assert chronology['first_difference']['k'] == chronology['first_admission']['k'] == 6
assert chronology['ledger']['inserts']-chronology['ledger']['removals'] == chronology['ledger']['remaining'] == 23
assert means['endpoint_exports'] == means['incoming_messages'] == 3541
assert means['aligned_candidates'] == 1743
for k in (6,21,24,25):
    e = next(e for e in edges if e['k'] == k)
    assert e['lifecycle']['marginal_keys'] == e['lifecycle']['retiring_factors'] == 0

# Host read-only checks: run outside the process-hiding sandbox.
containers = subprocess.check_output(['docker','inspect','-f','{{.Name}} {{.State.Status}}',
    'cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'],text=True).splitlines()
assert all(s.endswith(' exited') for s in containers), containers
processes = []
for name in ('launch.json','start.json'):
    receipt = json.loads((a.H49/name).read_text())
    pid = receipt['pid']; cmd = Path(f'/proc/{pid}/cmdline')
    value = cmd.read_bytes().replace(b'\0',b' ').decode(errors='replace') if cmd.exists() else None
    assert value is None or a.H49.name not in value, (pid,value)
    processes.append(dict(pid=pid, original_job_exited=True, current_command=value))

extra_code = [a.W/'src/glim_ros1/src/glim_ros/cbs_bridge.cpp',
              a.W/'src/Kimera-VIO/include/kimera-vio/imu-frontend/CbsNativeImuMeanBridge.h']
result = dict(passed=True, finished=datetime.datetime.now().astimezone().isoformat(),
    analyzed_inputs_unchanged=len(inputs), original_cpv49_outputs_unchanged=len(old),
    protected_files_verified=len(protected), original_report_inputs_verified=len(report_inputs),
    four_sequence_checkpoint=dict(cases=len(manifest['cases']), saved_files=len(manifest['files']), originals_also_verified=True),
    independent_SE3_cases=100, matrix_exponential_max_error=maximum, endpoint_composition_max_error=composition_maximum,
    candidate_joins=1743, ownership_ledger_updates=chronology['ledger']['updates'],
    containers=containers, processes=processes, source_replays=0, optimizer_runs=0,
    covariance_analysis=False, estimator_changes=False,
    separately_inspected_current_code_sha256={str(p):a.sha(p) for p in extra_code},
    output_sha256={str(p):a.sha(p) for p in a.H.iterdir() if p.is_file()})
a.save('verification.json', result)
print(json.dumps({k:v for k,v in result.items() if k not in ['output_sha256','separately_inspected_current_code_sha256']},indent=2))
