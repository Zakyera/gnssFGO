"""Bounded private diagnostic builds and saved-journal replays, create-once."""
import hashlib
import json
import os
from pathlib import Path
import resource
import subprocess
import sys
import time

HERE = Path(__file__).resolve().parent
ROOT = Path('/home/yeranis/results/gnssfgo_receiver_subphase_s5b_20261013')
BUILD = ROOT / 'build'
OLD = Path('/home/yeranis/results/gnssfgo_e1_e2_correctness_audit_20261011')
CAPTURE = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_shared_linearization_s5_live160_20261012/prepared')
RT = Path('/home/yeranis/results/gnssfgo_synthetic_uncertainty_20261004/baseline/runtime')
ENV = dict(LD_LIBRARY_PATH=str(RT/'lib'), OMP_NUM_THREADS='1',
  WNOJ_STRICT_PUBLICATION='1', WNOJ_INCREMENTAL_PROFILE='nominal', E2_EXACT_LINEARIZATION_CACHE='1')


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for b in iter(lambda: f.read(1<<20), b''): h.update(b)
    return h.hexdigest()


def main():
    tag = sys.argv[1]
    assert tag in ['timing_tests', 'unit', 'unit2', 'build', 'build2', 'prefix_reference', 'prefix_off', 'prefix_on',
                   'prefix_reference2', 'prefix_off2', 'prefix_on2', 'full_off', 'full_on']
    assert not (ROOT/(tag+'.json')).exists() and not (ROOT/(tag+'.log')).exists(), 'Refuse duplicate action'
    os.sched_setaffinity(0, {2})
    assert os.sched_getaffinity(0) == {2}
    env = os.environ | ENV
    for name in ['WNOJ_AUDIT_INDICES', 'CACHE_FAILURE_PROBE_DIR', 'E4_SELECTIVE_REFINEMENT',
                 'S5B_TIMING', 'S5B_PROFILE_OUTPUT']: env.pop(name, None)
    env.update(ASAN_OPTIONS='detect_leaks=0:halt_on_error=1', UBSAN_OPTIONS='halt_on_error=1:print_stacktrace=1')
    commands = []
    if tag == 'timing_tests':
        for name, flags in [('release', ['-O3']), ('sanitize', ['-O1', '-g1', '-fsanitize=address,undefined', '-fno-omit-frame-pointer'])]:
            binary = BUILD/('timing_'+name)
            assert not binary.exists()
            commands += [['c++', '-std=c++20', *flags, '-UNDEBUG', '-pthread', str(BUILD/'test_timing.cpp'), '-o', str(binary)],
              ['env', 'S5B_TIMING=0', str(binary)],
              ['env', 'S5B_TIMING=1', 'S5B_PROFILE_OUTPUT='+str(ROOT/('timing_'+name+'.csv')), str(binary)]]
    elif tag in ['unit', 'unit2', 'build', 'build2']:
        kind = 'unit' if tag.startswith('unit') else 'build'
        command = json.loads((OLD/('candidate_'+kind+'.json')).read_text())['commands'][0]
        command.insert(1, '-I'+str(BUILD))
        command = [str(BUILD/Path(a).name) if a in [str(OLD/'candidate'/n) for n in
          ['DiagnosticReplay.cpp','test_e2_audit.cpp','test_twist.cpp']] or
          (a.endswith('.cpp') and Path(a).name in ['test_cache.cpp', 'test_refinement.cpp']) else a for a in command]
        binary = BUILD/('replay' if kind=='build' else tag)
        assert not binary.exists()
        command[command.index('-o')+1] = str(binary)
        commands = [command]
        if kind == 'unit':
            commands += [['env', 'S5B_TIMING=0', str(binary), '--gtest_output=xml:'+str(ROOT/(tag+'_off.xml'))],
              ['env', 'S5B_TIMING=1', str(binary), '--gtest_output=xml:'+str(ROOT/(tag+'_on.xml'))]]
    else:
        for gate in ['timing_tests','unit2','build2']:
            assert json.loads((ROOT/(gate+'.json')).read_text())['passed'], gate
        if tag.startswith('full'):
            assert json.loads((ROOT/'prefix_validation.json').read_text())['passed']
        if tag == 'full_on':
            assert json.loads((ROOT/'full_off.json').read_text())['passed']
            assert json.loads((ROOT/'full_reference_validation.json').read_text())['passed']
        binary = OLD/'candidate/replay' if tag.startswith('prefix_reference') else BUILD/'replay'
        output = ROOT/tag
        assert not output.exists()
        env['S5B_TIMING'] = '1' if '_on' in tag else '0'
        env['S5B_PROFILE_OUTPUT'] = str(ROOT/(tag+'_profile.csv'))
        assert not Path(env['S5B_PROFILE_OUTPUT']).exists()
        journal = ROOT/'prefix5_valid.csv' if tag.startswith('prefix') else CAPTURE/'capacity160_on_receiver/events.csv'
        commands = [[str(binary),'--receiver-model','source_wnoa','--mode','sources','--timeline','record',
          '--solver','isam2','--support-policy','uniform_imu','--support-period-ns','100000000',
          '--receiver-startup-samples','59','--event-stream',str(journal),'--out',str(output)]]
    before = dict(load=os.getloadavg(), affinity=sorted(os.sched_getaffinity(0)))
    start = time.monotonic(); code = 0
    with (ROOT/(tag+'.log')).open('x') as log:
        for command in commands:
            try: code = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, env=env, timeout=1800).returncode
            except subprocess.TimeoutExpired: code = 124
            if code: break
    usage = resource.getrusage(resource.RUSAGE_CHILDREN)
    result = dict(passed=code==0, returncode=code, seconds=time.monotonic()-start, commands=commands,
      before=before, after=dict(load=os.getloadavg(), affinity=sorted(os.sched_getaffinity(0))),
      user_seconds=usage.ru_utime, system_seconds=usage.ru_stime, max_rss_kib=usage.ru_maxrss,
      environment={k:env[k] for k in sorted(env) if k.startswith(('WNOJ_','E2_','S5B_','OMP_','LD_LIBRARY_PATH'))})
    if tag not in ['timing_tests', 'unit'] and binary.exists(): result['binary_sha256'] = sha(binary)
    with (ROOT/(tag+'.json')).open('x') as f: json.dump(result,f,indent=2)
    print(tag, 'returncode', code, 'wall seconds', result['seconds'], flush=True)
    if code: print((ROOT/(tag+'.log')).read_text()[-8000:])
    raise SystemExit(code)


if __name__ == '__main__': main()
