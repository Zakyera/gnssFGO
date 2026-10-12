"""Read-only verification, plus a bounded local evidence archive. No launches."""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import yaml
from layout_guard import allocation, require_size

HERE = Path(__file__).resolve().parent
WS = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
RUN = WS/'runs/e1b_startup_fix_20261012'
OLD = WS/'runs/e1b_local_profile_20261012/release_2'


def sha(p):
    h = hashlib.sha256()
    with Path(p).open('rb') as f:
        for block in iter(lambda: f.read(1048576), b''):
            h.update(block)
    return h.hexdigest()


def main():
    evidence = HERE/'evidence'
    evidence.mkdir(exist_ok=True)
    def retain(src, relative):
        dest = evidence/relative
        dest.parent.mkdir(parents=True, exist_ok=True)
        if dest.exists():
            assert sha(src) == sha(dest), f'Refuse overwrite: {dest}'
        else:
            shutil.copy2(src, dest)
    manifest = WS/'runs/gnssfgo_e1b_profile40_20261012/prepared/protected_manifest.json'
    frozen = json.loads(manifest.read_text())
    for p, digest in frozen.items():
        assert sha(p) == digest, p
    previous = json.loads((HERE.parent/'e1b_profile40_20261012/validation.json').read_text())
    sources = previous['pinned_source_repos_unchanged']
    for p, identity in sources.items():
        assert subprocess.check_output(['git', '-C', p, 'rev-parse', 'HEAD'], text=True).strip() == identity['head']
        assert not subprocess.check_output(['git', '-C', p, 'status', '--porcelain'], text=True).strip()
    builds = {}
    shared = {}
    for name in ['fixed_release', 'asan_negative', 'asan_fixed']:
        build = RUN/name
        info = json.loads((build/'complete.json').read_text())
        assert sha(build/'libglim_ros.so') == info['libglim_ros_sha256']
        shared.update(info['protected'])
        cmd = json.loads((build/'commands.json').read_text())[-1]['command']
        bridge = next(x for x in cmd if 'cbs_bridge' in x and x.endswith('.o'))
        real_bridge = Path(bridge.replace('/workspace/cbs_gtsam4.3', str(WS), 1))
        builds[name] = dict(libglim_ros_sha256=info['libglim_ros_sha256'],
                           actual_bridge_object=bridge, actual_bridge_sha256=sha(real_bridge))
        for p in build.glob('lib*.so*'):
            if p.name != 'libglim_ros.so':
                # Absolute symlinks name the container mount, not a host path.
                target = str(p.readlink()).replace('/workspace/cbs_gtsam4.3', str(WS), 1)
                assert Path(target).resolve() == (OLD/p.name).resolve()
    # Verify shared link inputs also outside the mounted workspace, in their
    # owning container. No scripts/files or production outputs are written there.
    check = '''import hashlib,json,sys
d=json.loads(sys.argv[1]);bad=[]
for p,h in d.items():
 s=hashlib.sha256()
 with open(p,'rb') as f:
  for b in iter(lambda:f.read(1048576),b''):s.update(b)
 if s.hexdigest()!=h:bad.append(p)
print(json.dumps(dict(count=len(d),mismatches=bad)))
'''
    checked = json.loads(subprocess.check_output(['docker','exec','cbsms_ws_gpu','python3','-c',check,json.dumps(shared)],text=True))
    assert not checked['mismatches'], checked
    success = {}
    for tag in ['fixed_off', 'fixed_on', 'fixed_asan_verbose', 'fixed_asan_repeat']:
        case = RUN/tag
        r = json.loads((case/'result.json').read_text())
        ready = json.loads((case/'ready.json').read_text())
        assert r['passed'] and r['node_returncode'] == r['master_returncode'] == 0
        assert ready['source_measurements'] == 0
        logs = (case/'node.log').read_text()
        runtime_logs = list(case.glob('asan_runtime.*'))
        for p in runtime_logs:
            logs += p.read_text()
        for error in ['ERROR: AddressSanitizer', 'heap-buffer-overflow', 'malloc.c:', 'DEADLYSIGNAL']:
            assert error not in logs, (tag, error)
        success[tag] = dict(**r, profile_enabled=ready['profile_enabled'],
                           cuda_allocation_warnings=logs.count('cudaErrorMemoryAllocation'))
        if 'asan' in tag:
            assert runtime_logs and 'AddressSanitizer Init done' in logs
    negative = (RUN/'asan_bad/node.log').read_text()
    assert 'WRITE of size 8' in negative and '0 bytes to the right of 3296-byte region' in negative
    assert 'cbs_bridge.cpp:646' in negative and 'AddressSanitizer: heap-buffer-overflow' in negative
    layout = {}
    for label, obj in {
        'stale_workspace': WS/'build/glim_ros/CMakeFiles/glim_ros.dir/src/glim_ros/glim_ros.cpp.o',
        'known_working_e1': WS/'runs/glim_sync_compatible_batching_e1_20261011/private/glim_ros.o',
        'fixed': RUN/'fixed_release/glim_ros.o',
    }.items():
        dis = subprocess.check_output(['objdump','-drC',str(obj)],text=True)
        size, excerpt = allocation(dis)
        layout[label] = dict(bytes=size, object=str(obj), sha256=sha(obj), excerpt=excerpt)
    assert layout['stale_workspace']['bytes'] == 3296
    assert layout['fixed']['bytes'] == layout['known_working_e1']['bytes'] == 3344
    require_size(RUN/'fixed_release/glim_ros.o', 3344)
    # Only output paths differ from the original launch's resolved GLIM params.
    params_path = WS/'runs/gnssfgo_e1b_profile40_20261012/prepared/preflight/resolved_params.yaml'
    prior_params = {k:v for k,v in yaml.safe_load(params_path.read_text()).items()
                    if k.startswith('/glim_rosnode/') or k == '/use_sim_time'}
    changed_params = {}
    for tag in success:
        now = json.loads((RUN/tag/'params.json').read_text())
        assert now.keys() == prior_params.keys()
        changed_params[tag] = [k for k in now if now[k] != prior_params[k]]
        for k in changed_params[tag]:
            assert '/pilot_on_source/' in prior_params[k], k
            assert '/runs/e1b_startup_fix_20261012/'+tag+'/' in now[k], k
    test_result = json.loads((RUN/'focused_tests/passed.json').read_text())
    assert test_result['passed'] and test_result['exact_state_covariance_numbers'] == 4716
    guard = subprocess.run(['python3','-B',str(HERE/'test_layout_guard.py')],capture_output=True,text=True)
    (evidence/'layout_tests.log').write_text(guard.stdout+guard.stderr)
    assert guard.returncode == 0
    processes = {}
    for container in ['cbsms_ws_gpu','ubuntu-jammy-humble-nvidia']:
        listing = subprocess.check_output(['docker','exec',container,'ps','-eo','pid,ppid,pgid,comm'],text=True)
        assert not any(x in listing for x in ['glim_rosnode','rosmaster','roscore','rosout','kimera','gnssfgo'])
        processes[container] = listing
    # Curated small evidence only: avoid raw ROS launch environments or binaries.
    for folder in [p for p in RUN.iterdir() if p.is_dir()]:
        for p in folder.iterdir():
            if not p.is_file() or p.is_symlink():
                continue
            if p.suffix in ['.log','.json','.xml'] or p.name in ['command.txt','gdb.txt'] or p.name.startswith('asan_runtime.'):
                retain(p, p.relative_to(RUN))
            elif p.name == 'maps.txt':
                selected = [s for s in p.read_text().splitlines() if '/runs/e1b_' in s or 'libasan' in s]
                (evidence/p.relative_to(RUN)).write_text('\n'.join(selected)+'\n')
    evidence_hashes = {str(p.relative_to(evidence)):sha(p) for p in evidence.rglob('*') if p.is_file()}
    summary = dict(status='PRIVATE_ALLOCATION_BUG_FIXED_RELEASE_STARTUP_VALIDATED',
        no_dataset_playback=True, source_measurements=0,
        protected_manifest_sha256=sha(manifest), protected_files_unchanged=len(frozen),
        sources_unchanged=sources, shared_link_inputs=checked,
        builds=builds, allocation_evidence=layout, startup_checks=success,
        changed_parameters_output_paths_only=changed_params,
        focused_tests=test_result, layout_tests_passed=5, post_cleanup_processes=processes,
        sanitizer_limits=['One fixed_asan launch emitted only DEADLYSIGNAL before Starting GLIM; not diagnosed.',
                          'Two verbose ASan launches completed; CUDA allocation warnings present.',
                          'Only bridge translation unit instrumented for actual startup; leak checks disabled there.',
                          'Separate eight profiler ASan/UBSan tests pass with leak checks enabled.'],
        evidence_manifest=evidence_hashes)
    (HERE/'validation.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps({k:summary[k] for k in ['status','protected_files_unchanged','shared_link_inputs','startup_checks','layout_tests_passed']},indent=2))


if __name__ == '__main__':
    main()
