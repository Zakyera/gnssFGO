"""Finite authorized OFF/validate/ON/validate/score pipeline; never retries."""
from pathlib import Path
import datetime
import fcntl
import hashlib
import json
import os
import subprocess
import sys
import time

H = Path(__file__).resolve().parent
P = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_spx3_velodyne_full_cpv4_20261024/prepared')
V = Path('/home/yeranis/results/gnssfgo_spx3_velodyne_full_cpv4_20261024')
PY = '/home/yeranis/.local/share/pipx/venvs/evo/bin/python'
CONTAINERS = ('cbsms_ws_gpu', 'ubuntu-jammy-humble-nvidia')
stage = 'preflight'

def now(): return datetime.datetime.now().astimezone().isoformat()

def save(name, value):
    with (H/name).open('x') as f: json.dump(value, f, indent=2)

def status(state, **extra):
    data = dict(state=state, stage=stage, updated=now(), pid=os.getpid(), **extra)
    pending = H/'status.pending'
    pending.write_text(json.dumps(data, indent=2)+'\n')
    pending.replace(H/'status.json')
    print(json.dumps(data), flush=True)

def step(name, command):
    global stage
    stage = name; status('running')
    with (H/(name+'.log')).open('x') as log:
        subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, check=True)

def hashes():
    manifest = json.loads((P/'protected_manifest.json').read_text())
    for name, want in manifest.items():
        with Path(name).open('rb') as f: got = hashlib.file_digest(f, 'sha256').hexdigest()
        assert got == want, name
    return len(manifest)

def gate_paths(mode):
    return [P/f'full_{mode}_source/local_provenance.json',P/f'full_{mode}_source/velodyne_validation.json',
            P/f'full_{mode}_source/candidate_validation.json', V/mode/'validation.json']

def main():
    lock = (H/'pair.lock').open('a')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    assert not (H/'status.json').exists(), 'one attempt only; inspect existing status'
    assert not V.exists()
    assert not any(P.glob('full_*_control')) and not any(P.glob('full_*_source'))
    assert json.loads((P/'preflight/validation.json').read_text())['passed']
    count = hashes()
    inhibitors = subprocess.check_output(['systemd-inhibit','--list','--no-pager','--no-legend'], text=True)
    matches = [line for line in inhibitors.splitlines() if 'SPX3-CPV4' in line]
    assert len(matches) == 1 and 'block' in matches[0]
    assert all(token in matches[0] for token in ('sleep','idle','handle-lid-switch'))
    session_inhibitors = subprocess.check_output(['gnome-session-inhibit','--list'], text=True)
    session_matches = [line for line in session_inhibitors.splitlines() if line.startswith('SPX3-CPV4:')]
    assert len(session_matches) == 1 and all(t in session_matches[0] for t in ('suspend','idle'))
    initial = subprocess.check_output(['docker','inspect','-f','{{.State.Status}}',*CONTAINERS], text=True).split()
    assert initial == ['exited','exited'], ('unexpected initial containers', initial)
    save('start.json', dict(started=now(), pid=os.getpid(), original_containers=initial,
                           sleep_inhibitor=matches[0], session_inhibitor=session_matches[0], protected_files=count))
    started = False; failure = None
    try:
        started = True
        step('start_containers', ['docker','start',*CONTAINERS])
        for mode in ('off','on'):
            if mode == 'on':
                assert all(json.loads(path.read_text())['passed'] for path in gate_paths('off'))
                step('preflight_on', ['python3', str(P/'preflight_on.py')])
            runner = 'run_case.py' if mode == 'off' else 'run_case_on.py'
            step('capture_'+mode, ['python3', str(P/runner), 'full', mode])
            assert (P/f'full_{mode}_control/complete.json').exists()
            assert not (P/f'full_{mode}_control/failed.txt').exists()
            for name, arguments in [('sensor',['validate_velodyne.py',mode]),('local',['validate_local.py',mode]),
                                    ('candidate',['validate_candidate.py',mode]),
                                    ('receiver',['validate_case.py','check',mode])]:
                step('validate_'+mode+'_'+name, [PY,str(P/arguments[0]),*arguments[1:]])
            assert all(json.loads(path.read_text())['passed'] for path in gate_paths(mode))
            save(mode+'_validated.json', dict(passed=True, finished=now(), gates=list(map(str,gate_paths(mode)))))
        script = '/workspace/cbs_gtsam4.3/runs/gnssfgo_spx3_velodyne_full_cpv4_20261024/prepared/evaluate_fl4f.py'
        step('evaluate', ['docker','exec','cbsms_ws_gpu','bash','-lc',
                         'source /opt/ros/noetic/setup.bash && python3 '+script])
        step('summarize', ['python3',str(H/'summarize.py')])
        assert json.loads((H/'results.json').read_text())['passed']
    except BaseException as exc:
        failure = dict(stage=stage, error=repr(exc), at=now())
        save('failed.json', failure)
    finally:
        cleanup_error = None
        try:
            if started:
                # Source controller performs its own scoped cleanup. Wait for
                # it; do not stop a container while owned/unrelated jobs remain.
                deadline = time.monotonic()+60
                allowed = {'tail','zsh','ros2','rmw_zenohd'}
                while True:
                    top = {c:subprocess.check_output(['docker','top',c,'-eo','pid,comm,args'], text=True) for c in CONTAINERS}
                    extra = [line for value in top.values() for line in value.splitlines()[1:]
                             if line.strip() and line.split()[1] not in allowed]
                    if not extra: break
                    assert time.monotonic() < deadline, ('jobs remain; containers left running', extra)
                    time.sleep(2)
                with (H/'container_cleanup.log').open('x') as log:
                    subprocess.run(['docker','stop',*CONTAINERS],stdout=log,stderr=subprocess.STDOUT,check=True)
            final = subprocess.check_output(['docker','inspect','-f','{{.State.Status}}',*CONTAINERS], text=True).split()
            assert final == ['exited','exited'], final
            count = hashes()
            save('closeout.json', dict(passed=True,at=now(),containers=final,protected_files=count))
        except BaseException as exc:
            cleanup_error = repr(exc)
            save('cleanup_failed.json', dict(error=cleanup_error,at=now()))
        ok = failure is None and cleanup_error is None
        status('complete' if ok else 'failed', failure=failure, cleanup_error=cleanup_error)
        with (H/'REPORT.txt').open('a') as report:
            report.write('\nController finished '+now()+': '+('COMPLETED' if ok else 'STOPPED/FAILED')+'\n')
            report.write(json.dumps(dict(failure=failure,cleanup_error=cleanup_error))+'\n')
            if ok: report.write('Both modes and four gates each passed. See RESULTS_REPORT.txt/results.json for actual native/ONLINE/delayed metrics.\n')
            report.write('No automatic retry. Scope-limited inhibitor releases when controller exits.\n')
    if failure or cleanup_error: raise SystemExit(1)


if __name__ == '__main__': main()
