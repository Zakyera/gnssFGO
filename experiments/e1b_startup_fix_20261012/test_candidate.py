"""Run existing focused synthetic tests against corrected private DSOs.
Inside the existing Noetic container; no ROS/data/estimator experiment.
"""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

WS = Path('/workspace/cbs_gtsam4.3')
ROOT = WS/'runs/e1b_startup_fix_20261012'
OLD = WS/'runs/e1b_local_profile_20261012/release_2'


def main():
    out = ROOT/'focused_tests'
    out.mkdir(exist_ok=False)
    env = os.environ.copy()
    env.pop('CBS_E1B_PROFILE_PATH', None)
    env['OMP_NUM_THREADS'] = '1'
    env['LD_LIBRARY_PATH'] = f'{ROOT}/fixed_release:{OLD}:{WS}/devel/lib:' + env.get('LD_LIBRARY_PATH', '')
    env['ASAN_OPTIONS'] = 'detect_leaks=1:halt_on_error=1'
    env['UBSAN_OPTIONS'] = 'halt_on_error=1:print_stacktrace=1'
    commands = []
    def run(cmd, name):
        with (out/(name+'.log')).open('x') as f:
            result = subprocess.run(cmd, env=env, stdout=f, stderr=subprocess.STDOUT)
        commands.append(dict(command=cmd, returncode=result.returncode))
        (out/'commands.json').write_text(json.dumps(commands, indent=2))
        print(name, result.returncode, flush=True)
        assert result.returncode == 0, name
    counts = {}
    for name, exe in [('test_profile', OLD/'test_profile'),
                      ('test_solver', OLD/'test_solver'),
                      ('test_gpu', OLD/'test_gpu'),
                      ('asan_ubsan', OLD/'extra1/profile_asan_ubsan')]:
        run(['taskset', '-c', '2', str(exe), '--gtest_output=xml:'+str(out/(name+'.xml'))], name)
        xml = ET.parse(out/(name+'.xml')).getroot()
        counts[name] = {k: int(xml.attrib[k]) for k in ['tests', 'disabled', 'failures', 'errors']}
        assert counts[name]['failures'] == counts[name]['errors'] == 0
    run([str(OLD/'test_solver')], 'solver_repeat')
    def fingerprint(p):
        return next(line for line in p.read_text().splitlines() if line.startswith('E1B_MATH,'))
    first = fingerprint(out/'test_solver.log')
    assert first == fingerprint(out/'solver_repeat.log')
    assert first == fingerprint(OLD/'extra1/original_libraries.log')
    digest = hashlib.sha256(first.encode()).hexdigest()
    assert digest == 'f156d2b49e28f9d56f1ac6305c0a1ee1c7a0ef64d658c36ff6107d023b4e3d12'
    (out/'passed.json').write_text(json.dumps(dict(passed=True, counts=counts,
        exact_state_covariance_numbers=len(first.split(','))-3, fingerprint_sha256=digest,
        environment={k: env[k] for k in ['LD_LIBRARY_PATH','ASAN_OPTIONS','UBSAN_OPTIONS','OMP_NUM_THREADS']}), indent=2))


if __name__ == '__main__':
    main()
