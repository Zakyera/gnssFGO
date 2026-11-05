"""Finite saved-data scoring only. No source, replay, build or retry commands."""
from pathlib import Path
import datetime
import fcntl
import hashlib
import json
import os
import subprocess

Q = Path(__file__).resolve().parent
H = Q.parent
P = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_leica2_gravity_projection_cpv9_20261026/prepared')
V = Path('/home/yeranis/results/gnssfgo_leica2_gravity_projection_cpv9_20261026')
B = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_leica2_velodyne_full_cpv4_20261024/prepared')
BV = Path('/home/yeranis/results/gnssfgo_leica2_velodyne_full_cpv4_20261024')
PY = '/home/yeranis/.local/share/pipx/venvs/evo/bin/python'
CONTAINERS = ('cbsms_ws_gpu', 'ubuntu-jammy-humble-nvidia')

def now():
    return datetime.datetime.now().astimezone().isoformat()

def load(p):
    return json.loads(p.read_text())

def save(p, value):
    with p.open('x') as f:
        json.dump(value, f, indent=2)

def sha(p):
    with p.open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()

def check_hashes(manifest):
    for path, want in manifest.items():
        assert sha(Path(path)) == want, path

def states():
    return subprocess.check_output(['docker', 'inspect', '-f', '{{.State.Status}}',
                                    *CONTAINERS], text=True).split()

def step(name, command):
    print(now(), name, flush=True)
    with (Q/(name+'.log')).open('x') as f:
        subprocess.run(command, stdout=f, stderr=subprocess.STDOUT, check=True)
    save(Q/(name+'.done.json'), dict(passed=True, finished=now(), command=command))

def main():
    lock = (H/'pair.lock').open('a')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    assert not (Q/'start.json').exists(), 'Do not repeat this reporting attempt'
    assert not (P/'evaluation_full').exists()
    assert not (H/'results.json').exists() and not (H/'decision.json').exists()
    assert load(H/'closeout.json')['passed']
    assert load(H/'status.json')['stage'] == 'validate_on_projection'
    assert load(Q/'audit.json')['passed']
    assert load(P/'full_on_source/projection_validation.json')['passed']
    manifest = load(P/'protected_manifest.json')
    inputs = load(Q/'audit.json')['input_sha256']
    check_hashes(manifest); check_hashes(inputs)
    for root, validation, mode in ((B, BV, 'off'), (B, BV, 'on'), (P, V, 'on')):
        assert (root/f'full_{mode}_control/complete.json').is_file()
        assert not (root/f'full_{mode}_control/failed.txt').exists()
        for path in [root/f'full_{mode}_source'/name for name in
                     ('velodyne_validation.json', 'local_provenance.json', 'candidate_validation.json')]+[validation/mode/'validation.json']:
            assert load(path)['passed'], str(path)
    initial = states()
    assert initial == ['exited', 'exited'], initial
    save(Q/'start.json', dict(pid=os.getpid(), at=now(), initial_containers=initial,
                             protected_files=len(manifest), no_source_rerun=True))
    started = False; failure = None; cleanup_error = None
    try:
        started = True
        step('start_noetic', ['docker', 'start', 'cbsms_ws_gpu'])
        script = '/workspace/cbs_gtsam4.3/runs/gnssfgo_leica2_gravity_projection_cpv9_20261026/prepared/evaluate_fl4f.py'
        step('evaluate', ['docker', 'exec', 'cbsms_ws_gpu', 'bash', '-lc',
                         'source /opt/ros/noetic/setup.bash && python3 '+script])
        step('summarize', ['python3', str(H/'summarize.py')])
        assert load(H/'results.json')['passed']
        step('decision', [PY, str(H/'decision.py')])
        assert load(H/'decision.json')['passed']
    except BaseException as exc:
        failure = repr(exc)
        save(Q/'failed.json', dict(error=failure, at=now()))
    finally:
        try:
            if started:
                top = subprocess.check_output(['docker', 'top', 'cbsms_ws_gpu',
                                                '-eo', 'pid,comm,args'], text=True)
                save(Q/'cleanup_processes.json', dict(at=now(), top=top))
                extra = [line for line in top.splitlines()[1:]
                         if line.strip() and line.split()[1] not in {'tail', 'zsh'}]
                assert not extra, ('Jobs remain; container not stopped', extra)
                step('stop_noetic', ['docker', 'stop', 'cbsms_ws_gpu'])
            assert states() == ['exited', 'exited']
            check_hashes(manifest); check_hashes(inputs)
        except BaseException as exc:
            cleanup_error = repr(exc)
        ok = failure is None and cleanup_error is None
        result = dict(passed=ok, at=now(), failure=failure, cleanup_error=cleanup_error,
                      protected_unchanged=len(manifest), input_unchanged=len(inputs),
                      containers=states(), no_source_rerun=True,
                      original_controller_failure_preserved=True)
        if (H/'decision.json').exists():
            result['scientific_screen_passed'] = load(H/'decision.json')['scientific_screen_passed']
        save(Q/'final.json', result)
        print(json.dumps(result, indent=2), flush=True)
    if not ok:
        raise SystemExit(1)

if __name__ == '__main__':
    main()
