"""Once-only private saved first-factor A/B experiment; no sensor controller."""
import datetime
import hashlib
import json
import os
import subprocess
import time
import traceback
from pathlib import Path

H = Path(__file__).resolve().parent
W = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
P = W / 'runs' / H.parent.name / H.name
Q = W / 'runs/eig2_hesai_insertion_cpv51_20261030'
H51 = H.parent.parent / Q.name
I = '/workspace/cbs_gtsam4.3/runs/' + H.parent.name + '/' + H.name
J = '/workspace/cbs_gtsam4.3/runs/' + Q.name
K = '/workspace/cbs_gtsam4.3/runs/kimera_on_incremental_refresh_cpv40_20261029'

def save(name, data):
    with (H / name).open('x') as stream:
        json.dump(data, stream, indent=2)

def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()

def check(pins):
    for path, expected in pins.items():
        assert sha(path) == expected, path

def call(args):
    return subprocess.check_output(args, text=True)

def run(label, args):
    with (P / (label + '.log')).open('x') as stream:
        job = subprocess.Popen(args, stdout=stream, stderr=subprocess.STDOUT)
        save(label + '_process.json', dict(pid=job.pid, args=args, time=time.time()))
        rc = job.wait()
    assert rc == 0, (label, rc)

save('start.json', dict(pid=os.getpid(), time=datetime.datetime.now().astimezone().isoformat()))
pins = json.loads((Q / 'prepared/protected_manifest.json').read_text())
pins.update(json.loads((H51 / 'complete.json').read_text())['output_sha256'])
pins[str(H51 / 'complete.json')] = sha(H51 / 'complete.json')
pins.update(json.loads((H.parent / 'protected_manifest.json').read_text()))
for path in H.parent.glob('*'):
    if path.is_file():
        pins[str(path)] = sha(path)
for root in (H, P):
    for path in root.rglob('*'):
        if path.is_file() and path != H / 'start.json':
            pins[str(path)] = sha(path)
save('protected_manifest.json', pins)
check(pins)
states = call(['docker', 'inspect', '-f', '{{.Name}} {{.State.Status}}',
               'cbsms_ws_gpu', 'ubuntu-jammy-humble-nvidia']).splitlines()
assert states == ['/cbsms_ws_gpu exited', '/ubuntu-jammy-humble-nvidia exited'], states
free = os.statvfs(W).f_bavail * os.statvfs(W).f_frsize
assert free > 4 * 1024 ** 3, free
save('preflight.json', dict(protected=len(pins), containers=states, free_bytes=free))
try:
    subprocess.run(['docker', 'start', 'cbsms_ws_gpu'], check=True)
    commands = json.loads((W / 'runs/kimera_on_incremental_refresh_cpv40_20261029/actual_test_commands.json').read_text())
    for mode in ('release', 'san'):
        for kind, original in commands.items():
            args = []
            for item in original:
                if item in ('-I' + K, '-I' + K + '/include'):
                    item = item.replace(K, J)
                if item == K + '/test.cpp':
                    item = I + '/FirstFactor.cpp'
                elif item == K + '/test.o':
                    item = I + '/' + mode + '.o'
                elif item == K + '/test':
                    item = I + '/' + mode
                args.append(item)
            if mode == 'san':
                args = [item for item in args if item != '-O3']
                flags = ['-O1', '-g1', '-fsanitize=address,undefined',
                         '-fno-sanitize-recover=all', '-fno-omit-frame-pointer']
                flags += ['-fno-pie', '-DEIGEN_MALLOC_ALREADY_ALIGNED=1'] if kind == 'compile' else ['-no-pie']
                args[1:1] = flags
            save(mode + '_' + kind + '_command.json', args)
            run(mode + '_' + kind, ['docker', 'exec', 'cbsms_ws_gpu', *args])
        env = ['-e', 'LD_LIBRARY_PATH=' + K + '/private_build:/workspace/cbs_gtsam4.3/devel/lib']
        if mode == 'san':
            env += ['-e', 'ASAN_OPTIONS=detect_leaks=0:halt_on_error=1', '-e', 'UBSAN_OPTIONS=halt_on_error=1']
        run(mode + '_selftest', ['docker', 'exec', *env, 'cbsms_ws_gpu', I + '/' + mode, 'selftest'])
    run('test', ['docker', 'exec', '-e', 'LD_LIBRARY_PATH=' + K + '/private_build:/workspace/cbs_gtsam4.3/devel/lib',
                 'cbsms_ws_gpu', I + '/release', J + '/prepared/full_on_source/checkpoints',
                 J + '/prepared/full_off_source/checkpoints', I + '/results'])
    for phase in ('ordinary', 'refresh'):
        original = Q / 'prepared/full_on_source/checkpoints/update_6' / ('after_' + phase) / 'checkpoint.bin'
        assert sha(P / 'results' / (phase + '_before.bin')) == sha(P / 'results' / (phase + '_after.bin')) == sha(original)
    assert sha(P / 'results/B_target_restored.bin') == sha(P / 'results/ordinary_before.bin')
    check(pins)
    save('test_passed.json', dict(passed=True, protected=len(pins), snapshot_nonmutation=2,
                                  only_target_removed=True, native_GN_steps=4,
                                  finished=datetime.datetime.now().astimezone().isoformat()))
except Exception:
    save('failure.json', dict(traceback=traceback.format_exc(), time=time.time()))
    raise
finally:
    top = call(['docker', 'top', 'cbsms_ws_gpu', '-eo', 'pid,comm,args'])
    assert all(row.split()[1] in ('tail', 'zsh') for row in top.splitlines()[1:] if row.strip()), top
    subprocess.run(['docker', 'stop', 'cbsms_ws_gpu'], check=True)
    states = call(['docker', 'inspect', '-f', '{{.Name}} {{.State.Status}}',
                   'cbsms_ws_gpu', 'ubuntu-jammy-humble-nvidia']).splitlines()
    save('cleanup.json', dict(jobs=top, containers=states, time=time.time()))
