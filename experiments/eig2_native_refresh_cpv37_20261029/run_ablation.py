"""Finite offline-only CP-V37 controller; no ROS/source/receiver launch."""
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import time

H = Path(__file__).resolve().parent
W = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
OLD = W / 'runs/eig2_counterfactual_cpv36_20261029'
OH = H.parent / 'eig2_counterfactual_cpv36_20261029'
P = W / 'runs/eig2_native_refresh_cpv37_20261029'
Q = '/workspace/cbs_gtsam4.3/runs/eig2_native_refresh_cpv37_20261029'
OQ = '/workspace/cbs_gtsam4.3/runs/eig2_counterfactual_cpv36_20261029'
CONTAINER = 'cbsms_ws_gpu'


def sha(path):
    with Path(path).open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()


def save(path, value):
    with path.open('x') as f:
        json.dump(value, f, indent=2)


def check(pins):
    for path, digest in pins.items():
        assert sha(path) == digest, path


def command(args, log, timeout=180):
    start = time.monotonic()
    with log.open('x') as f:
        p = subprocess.Popen(args, stdout=f, stderr=subprocess.STDOUT)
        save(log.with_suffix(log.suffix + '.process.json'), dict(pid=p.pid, argv=args))
        try:
            code = p.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            p.terminate()
            p.wait(timeout=15)
            raise
    save(log.with_suffix(log.suffix + '.receipt.json'),
         dict(returncode=code, seconds=time.monotonic()-start, pid=p.pid))
    print(log.name, code, round(time.monotonic()-start, 2), flush=True)
    assert code == 0, str(log)


def container_states():
    return subprocess.check_output([
        'docker', 'inspect', '--format', '{{.Name}} {{.State.Running}}',
        CONTAINER, 'ubuntu-jammy-humble-nvidia'], text=True).splitlines()


def env_args():
    return ['docker', 'exec', '-e', 'LD_LIBRARY_PATH='+OQ+'/private_build:/workspace/cbs_gtsam4.3/devel/lib:/usr/local/lib',
            '-e', 'ASAN_OPTIONS=detect_leaks=0:halt_on_error=1',
            '-e', 'UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1', CONTAINER]


assert not (H/'start.json').exists(), 'Refuse duplicate controller'
assert container_states() == ['/cbsms_ws_gpu false', '/ubuntu-jammy-humble-nvidia false']
assert shutil.disk_usage(W).free > 8*1024**3
assert json.loads((OH/'closeout.json').read_text())['passed']
pins = json.loads((OH/'protected_before.json').read_text())
check(pins)
private = {str(OLD/k): v for k, v in json.loads((OLD/'prepared_manifest.json').read_text()).items()}
check(private)
pins.update(private)
for directory in [OH, OLD/'capture/snapshots', OLD/'include', OLD/'build_release_native2/snapshots/update_11']:
    for file in directory.rglob('*'):
        if file.is_file():
            pins[str(file)] = sha(file)
for name in ['Checkpoint.h', 'Binary.h', 'OriginalChartPrior.h', 'actual_test_commands.json']:
    pins[str(OLD/name)] = sha(OLD/name)
for name in ['all_method1_rerun_cpv33_20261029', 'all_method2_rerun_cpv30_20261029']:
    c = json.loads((H.parent/name/'closeout.json').read_text())
    old_outputs = c.get('output_sha256', c.get('outputs'))
    check(old_outputs)
    pins.update(old_outputs)
save(H/'protected_before.json', pins)
save(H/'implementation_before.json', {str(f): sha(f) for f in [P/'NativeRefresh.cpp', H/'PROTOCOL.txt', Path(__file__)]})
save(H/'start.json', dict(pid=os.getpid(), wall_ns=time.time_ns(), checkpoints=[800,934,1000,1335,1336,1337,1338], source_runs=0))
completed = False
try:
    subprocess.run(['docker', 'start', CONTAINER], check=True)
    for variant in ['release', 'sanitizer']:
        D = P/('build_'+variant)
        D.mkdir(exist_ok=False)
        commands = json.loads((OLD/'actual_test_commands.json').read_text())
        for name, args in commands.items():
            args = [a.replace(OQ+'/observer_test.cpp', Q+'/NativeRefresh.cpp')
                    .replace(OQ+'/test.o', Q+'/build_'+variant+'/test.o')
                    if a != OQ+'/test' else Q+'/build_'+variant+'/test' for a in args]
            if variant == 'sanitizer':
                args = [a for a in args if a != '-O3']
                args += ['-O1', '-g1', '-fsanitize=address,undefined', '-fno-sanitize-recover=all', '-fno-omit-frame-pointer']
                args += ['-fno-pie', '-DEIGEN_MALLOC_ALREADY_ALIGNED=1'] if name == 'compile' else ['-no-pie']
            commands[name] = args
        save(D/'commands.json', commands)
        for name, args in commands.items():
            command(['docker','exec',CONTAINER]+args, D/(name+'.log'))
        command(env_args()+[Q+'/build_'+variant+'/test', 'fixture', OQ+'/build_release_native2/snapshots/update_11/checkpoint.bin', Q+'/build_'+variant+'/fixture'], D/'fixture.log')
        for suffix in ['', '_displaced']:
            f = D/('fixture'+suffix)
            assert sha(f/'input_before.bin') == sha(f/'input_after.bin')
            assert all((f/(branch+'_complete.txt')).exists() for branch in ['stored_control','refreshed_once'])
        save(D/'passed.json', dict(passed=True, original_and_displaced=True, vendor_libraries_sanitized=False))
    command(env_args()+['ldd', Q+'/build_release/test'], H/'ldd.log')
    ldd = (H/'ldd.log').read_text()
    assert OQ+'/private_build/libcbs.so' in ldd and 'not found' not in ldd
    save(H/'ready.json', dict(release=True, sanitizer=True, executable_sha256=sha(P/'build_release/test'), no_production_change=True))
    for k in [800,934,1000,1335,1336,1337,1338]:
        assert shutil.disk_usage(W).free > 5*1024**3
        command(env_args()+[Q+'/build_release/test', 'data', OQ+'/capture/snapshots/update_'+str(k)+'/checkpoint.bin', Q+'/results/update_'+str(k)], H/('checkpoint_'+str(k)+'.log'))
        d = P/'results'/('update_'+str(k))
        assert sha(d/'input_before.bin') == sha(d/'input_after.bin')
    check(pins)
    completed = True
    save(H/'finished.json', dict(passed=True, comparison_completed=True, source_runs=0, outcomes_require_interpretation=True))
except Exception as e:
    save(H/'failed.json', dict(error=repr(e), source_runs=0))
    raise
finally:
    top = subprocess.check_output(['docker','top',CONTAINER,'-eo','pid,comm'], text=True)
    # Refuse to stop a container that still has an unrelated/owned command.
    assert all(line.split()[-1]=='tail' for line in top.splitlines()[1:]), top
    subprocess.run(['docker','stop','-t','20',CONTAINER],check=True)
    states = container_states()
    save(H/'restoration.json', dict(states=states, top=top, comparison_completed=completed))
    assert states == ['/cbsms_ws_gpu false', '/ubuntu-jammy-humble-nvidia false']
