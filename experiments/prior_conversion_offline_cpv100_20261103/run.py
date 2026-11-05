"""One isolated offline diagnostic; no source/receiver/optimizer launch."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time

H = Path(__file__).resolve().parent
E = H.parent
W = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
K = W / 'runs' / H.name
C = W / 'runs/persistent_compatibility_cpv76_20261101'
F = C / 'release_fixture1'
S = F / 'source_snapshot'
Q = W / 'runs/eth1_early_native_capture_cpv98_20261103/prepared/on_source/checkpoints'
P99 = W / 'runs/eth1_split_imu_repeat_cpv99_20261103/prepared'
H99 = E / 'eth1_split_imu_repeat_cpv99_20261103'

def read(p):
    return json.loads(p.read_text())

def sha(p):
    with Path(p).open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()

def save(name, value):
    with (H / name).open('x') as f:
        json.dump(value, f, indent=2)

def call(args):
    return subprocess.check_output(args, text=True)

def inside(p):
    return str(p).replace(str(W), '/workspace/cbs_gtsam4.3')

def verify(pins):
    for p, expected in pins.items():
        assert sha(p) == expected, p
    return len(pins)

def proc_start(pid):
    p = Path(f'/proc/{pid}/stat')
    return p.read_text().rsplit(')', 1)[1].split()[19] if p.exists() else None

def run(name, args, timeout=300):
    with (H / (name + '.log')).open('x') as f:
        job = subprocess.Popen(args, stdout=f, stderr=subprocess.STDOUT)
        save(name + '_process.json', dict(pid=job.pid, proc_start=proc_start(job.pid), args=args))
        try:
            rc = job.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            # Only this diagnostic's owned docker-exec client; do not restart.
            job.terminate()
            job.wait()
            raise
    save(name + '_receipt.json', dict(exit=rc))
    assert rc == 0, (name, rc)

assert not (H / 'start.json').exists(), 'one-shot diagnostic already started'
before = call(['docker', 'inspect', '-f', '{{.Name}} {{.State.Status}}',
               'cbsms_ws_gpu', 'ubuntu-jammy-humble-nvidia'])
assert before == '/cbsms_ws_gpu exited\n/ubuntu-jammy-humble-nvidia exited\n'
old_audit = read(H99 / 'failure_post_closeout.json')
for job in old_audit['owned_jobs']:
    assert proc_start(job['pid']) != job['recorded_start'], job
pins = read(P99 / 'protected_manifest.json')
assert len(pins) == 6425
pins.update(old_audit['terminal_sha256'])
for name in ('eth1_early_native_capture_cpv98_20261103',
             'eth1_split_imu_live_pair_cpv95_20261103_continuation1',
             'eth1_split_imu_persistent_on_cpv91_20261102_attempt1'):
    pins.update(read(E / name / 'complete.json')['output_sha256'])
source_pins = read(F / 'sources.json')
for path in S.rglob('*'):
    if path.is_file():
        original = inside(C / path.relative_to(S))
        if original in source_pins:
            assert sha(path) == source_pins[original], path
        pins[str(path)] = sha(path)
for path in [*Q.rglob('checkpoint.bin'), K / 'probe.cpp', H / 'PROTOCOL.txt', H / 'run.py',
             W / 'devel/lib/libgtsam.so.4.3a1', W / 'devel/lib/libgtsam_unstable.so.4.3a1',
             F / 'compatibility_compile_command.json', F / 'compatibility_link_command.json']:
    pins[str(path)] = sha(path)
verify(pins)
save('inputs.json', pins)
save('start.json', dict(pid=os.getpid(), proc_start=proc_start(os.getpid()),
                        time=time.time(), containers=before, protected=6425,
                        input_files=len(pins), source_runs=0, estimator_builds=0))
failure = None
try:
    run('start_container', ['docker', 'start', 'cbsms_ws_gpu'], 60)
    deadline = time.monotonic() + 30
    while True:
        top = call(['docker', 'top', 'cbsms_ws_gpu', '-eo', 'pid,comm,args'])
        if all(row.split()[1] in {'tail', 'bash', 'zsh'} for row in top.splitlines()[1:] if row.strip()):
            break
        assert time.monotonic() < deadline, top
        time.sleep(1)
    save('initial_jobs.json', dict(top=top))
    for kind in ('compile', 'link'):
        args = read(F / ('compatibility_' + kind + '_command.json'))
        replacement = {
            '-I' + inside(C): '-I' + inside(S),
            '-I' + inside(C / 'include'): '-I' + inside(S / 'include'),
            inside(C / 'persistent_test.cpp'): inside(K / 'probe.cpp'),
            inside(F / 'compatibility.o'): inside(K / 'probe.o'),
            inside(F / 'compatibility'): inside(K / 'probe')}
        args = [replacement.get(a, a) for a in args]
        if kind == 'compile':
            args += ['-MMD', '-MF', inside(K / 'dependencies.d')]
        save(kind + '_command.json', args)
        run(kind, ['docker', 'exec', 'cbsms_ws_gpu', *args])
    env = ['docker', 'exec', '-e', 'LD_LIBRARY_PATH=' + inside(C / 'private_build') + ':' + inside(W / 'devel/lib'), 'cbsms_ws_gpu']
    run('probe', env + [inside(K / 'probe'), inside(Q), inside(K / 'results')])
    verified = verify(pins)
    save('diagnostic_passed.json', dict(passed=True, inputs_verified=verified,
                                      source_runs=0, optimizer_runs=0, estimator_builds=0))
except BaseException as exc:
    failure = repr(exc)
    save('failed.json', dict(error=failure))
finally:
    top = call(['docker', 'top', 'cbsms_ws_gpu', '-eo', 'pid,comm,args'])
    save('final_jobs.json', dict(top=top))
    run('restore_container', ['docker', 'stop', '--time', '20', 'cbsms_ws_gpu'], 60)
    after = call(['docker', 'inspect', '-f', '{{.Name}} {{.State.Status}}',
                  'cbsms_ws_gpu', 'ubuntu-jammy-humble-nvidia'])
    assert before == after
    jobs = [read(p) for p in H.glob('*_process.json')]
    assert all(proc_start(r['pid']) != r['proc_start'] for r in jobs)
    save('cleanup.json', dict(containers=after, owned_jobs_exited=True,
                             inputs_verified=verify(pins), no_sensor_or_optimizer_runs=True))
if failure:
    raise SystemExit(failure)
print((H / 'probe.log').read_text())
print('Offline diagnostic complete; original files verified, containers restored stopped.')
