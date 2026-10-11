#!/usr/bin/env python3
"""Read-only runtime preflight; --freeze writes NEW preparation records only.

No ROS master, bag, estimator, receiver, optimization or replay is started.
"""
import argparse
import ast
import difflib
import hashlib
import json
import shlex
import socket
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
import yaml
from experiment import ROOT, REPO, BASE, PROFILE, BINARY, inside, launch_arguments

def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for b in iter(lambda: f.read(1024 * 1024), b''):
            h.update(b)
    return h.hexdigest()

def exclusive(path, value):
    with path.open('x') as f:
        f.write(value)

def docker_ros(command):
    setup = 'source /opt/ros/noetic/setup.bash && source /workspace/cbs_gtsam4.3/devel/setup.bash && '
    return subprocess.check_output(['docker', 'exec', 'cbsms_ws_gpu', 'bash', '-lc',
                                    setup + shlex.join(command)], text=True)

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--freeze', action='store_true')
    args = p.parse_args()
    assert not any((ROOT / ('pilot_on_' + x)).exists() for x in ['source', 'receiver', 'control'])
    protected = json.loads((BASE / 'protected_manifest_v2.json').read_text())
    for path, value in protected.items():
        assert sha(path) == value, ('frozen baseline changed', path)
    assert sha(BINARY) == 'fb56ffc5c6c9cb575d6129289688caeabb4e3baba32e429b857f3d6a30e50d1f'
    for script in ROOT.glob('*.py'):
        ast.parse(script.read_text(), filename=str(script))
    for script in [ROOT / 'source_runner.sh', ROOT / 'source.env']:
        subprocess.run(['bash', '-n', str(script)], check=True)
    ET.parse(ROOT / 'active_window.launch')
    sys.path.insert(0, str(REPO / 'tools'))
    from live_receiver_protocol import validate_config
    config = json.loads((ROOT / 'pilot_on.json').read_text())
    validate_config(config)
    old = json.loads((BASE / 'bounded120_on.json').read_text())
    diff = {k: [old.get(k), config.get(k)] for k in set(old) | set(config) if old.get(k) != config.get(k)}
    assert set(diff) == {'sequence', 'ros_master_uri', 'sources'}
    for a, b in zip(old['sources'], config['sources']):
        assert {k:v for k,v in a.items() if k != 'session'} == {k:v for k,v in b.items() if k != 'session'}
    images = {}
    for container in ['cbsms_ws_gpu', 'ubuntu-jammy-humble-nvidia']:
        state = subprocess.check_output(['docker', 'inspect', '--format', '{{.State.Running}} {{.Image}}', container], text=True).strip()
        assert state.startswith('true '), state
        images[container] = state
    with socket.socket() as sock:
        assert sock.connect_ex(('127.0.0.1', 11388)) != 0, 'owned ROS port already occupied'
    launch = inside(ROOT / 'active_window.launch')
    params = docker_ros(['roslaunch', '--dump-params', launch, *launch_arguments()])
    kimera_args = docker_ros(['roslaunch', '--args=/kimera_vio_ros/kimera_vio_ros_node', launch, *launch_arguments()])
    glim_args = docker_ros(['roslaunch', '--args=/glim_rosnode', launch, *launch_arguments()])
    launch_files = docker_ros(['roslaunch', '--files', launch, *launch_arguments()])
    for line in launch_files.splitlines():
        if line.startswith('/workspace/'):
            path = Path('/home/yeranis/repos/V4RL') / line.removeprefix('/workspace/')
            protected[str(path)] = sha(path)
    mapping = yaml.safe_load(params)
    assert mapping['/glim_rosnode/glim_cbs_mode'] == 'active_window_temporary'
    assert mapping['/glim_rosnode/glim_cbs_live_revision_enable'] is False
    assert mapping['/glim_rosnode/glim_cbs_health_aware_enable'] is False
    assert mapping['/glim_rosnode/glim_cbs_k2g_odom_factor_covariance_scale'] == 1.0
    assert '--cbs_odom_factor_mode=active_window_temporary' in kimera_args
    assert '--cbs_use_temporary_cbs_linear_factors=false' in kimera_args
    assert '--cbs_live_revision_enable=false' in kimera_args
    assert '--cbs_l2k_odom_factor_covariance_scale=1.0' in kimera_args
    assert '--cbs_health_aware_enable=false' in kimera_args
    assert '--cbs_use_local_smoother_for_belief_covariance=true' in kimera_args
    assert '--cbs_use_persistent_bpsam_sidecar_for_belief_covariance=false' in kimera_args
    # No estimator execution: inspect linked binary bytes for compiled-in mode.
    ws = ROOT.parents[1]
    source_runtime = [ws/'devel/lib/glim_ros/glim_rosnode',
        ws/'devel/lib/kimera_vio_ros/kimera_vio_ros_node',
        ws/'devel/lib/libglim_ros.so', ws/'devel/lib/libkimera_vio.so',
        ws/'devel/lib/libkimera_vio_ros.so', ws/'devel/lib/libcbs.so',
        ws/'devel/lib/libgtsam.so.4', ws/'devel/lib/libgtsam_unstable.so.4']
    for name in ['libglim_ros.so', 'libkimera_vio.so']:
        assert b'active_window_temporary' in (ws/'devel/lib'/name).read_bytes(), name
    assert b'active_window_remove_committed' in (ws/'devel/lib/libcbs.so').read_bytes()
    for path in source_runtime:
        protected[str(path)] = sha(path)
    dataset = ws / 'src/datasets/GrandTour'
    bags = {}
    for suffix in ['hesai', 'cpt7_imu', 'alphasense', 'ap20_prism_position', 'cpt7_ie_tc']:
        bag = dataset / ('2024-10-01-11-29-55_' + suffix + '.bag')
        assert bag.is_file()
        bags[str(bag)] = dict(bytes=bag.stat().st_size, sha256=sha(bag))
        protected[str(bag)] = bags[str(bag)]['sha256']
    source_repos = [REPO] + [ws/'src'/name for name in ['cbs', 'cbsms', 'glim', 'glim_ros1', 'Kimera-VIO', 'Kimera-VIO-ROS']]
    provenance = {}
    for repo in source_repos:
        provenance[str(repo)] = dict(head=subprocess.check_output(['git','-C',str(repo),'rev-parse','HEAD'],text=True).strip(),
            status=subprocess.check_output(['git','-C',str(repo),'status','--short'],text=True),
            diff_sha256=hashlib.sha256(subprocess.check_output(['git','-C',str(repo),'diff','HEAD'])).hexdigest())
    originals = {
        ROOT/'source_runner.sh': ws/'src/cbsms/tools/run_grandtour_eth1_glim_kimera.sh',
        ROOT/'active_window.launch': ws/'src/glim_ros1/launch/grandtour_eth1_glim_kimera_cbs.launch',
        ROOT/'run_case.py': BASE/'run_case.py',
        ROOT/'source.env': BASE/'source.env'}
    differences = ''
    for prepared, original in originals.items():
        protected[str(original)] = sha(original)
        differences += ''.join(difflib.unified_diff(original.read_text().splitlines(True), prepared.read_text().splitlines(True), fromfile=str(original), tofile=str(prepared)))
    for path in ROOT.iterdir():
        if path.is_file() and path.name != 'protected_manifest.json':
            protected[str(path)] = sha(path)
    result = dict(passed=True, status='PREPARED_NOT_RUN', static_launch_resolved=True,
        estimator_or_receiver_started=False, ros_master_started=False,
        duration_s=40, bag_offset_s=0, playback_rate=.1, receiver_changes=diff,
        source_modes=['active_window_temporary']*2, containers=images,
        source_runtime_hashes={str(p):sha(p) for p in source_runtime}, bags=bags, git=provenance,
        runtime_limit='Static preflight only. Effective live flags, removals, output coverage and receiver convergence remain to be checked after launch.')
    if args.freeze:
        out = ROOT / 'preflight'
        out.mkdir(exist_ok=False)
        for name, value in [('resolved_params.yaml', params), ('kimera_args.txt', kimera_args),
                            ('glim_args.txt', glim_args), ('launch_files.txt', launch_files), ('isolated_changes.diff', differences),
                            ('validation.json', json.dumps(result, indent=2)+'\n')]:
            exclusive(out/name, value)
        exclusive(ROOT/'protected_manifest.json', json.dumps(protected,indent=2)+'\n')
    else:
        for path, expected in json.loads((ROOT/'protected_manifest.json').read_text()).items():
            assert sha(path) == expected, path
    print(json.dumps({k:result[k] for k in ['status','static_launch_resolved','estimator_or_receiver_started','duration_s','playback_rate']},indent=2))

if __name__ == '__main__':
    main()
