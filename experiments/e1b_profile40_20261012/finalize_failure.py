"""Preserve one failed pre-playback attempt; never launch/retry an estimator."""
import ast
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

HERE=Path(__file__).resolve().parent
WS=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
RUN=WS/'runs/gnssfgo_e1b_profile40_20261012'
ROOT=RUN/'prepared'
SOURCE=ROOT/'profile40_on_source'
RECEIVER=ROOT/'profile40_on_receiver'
CONTROL=ROOT/'profile40_on_control'
OUT=HERE/'evidence'
OUT.mkdir(exist_ok=True)
assert not list(OUT.iterdir()), 'Refuse replacing archived evidence'
def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()
manifest=json.loads((ROOT/'protected_manifest.json').read_text())
for path,h in manifest.items(): assert sha(Path(path))==h,path
previous=json.loads((HERE.parent/'e1b_local_profile_20261012/validation.json').read_text())
for repo,v in previous['source_git_unchanged'].items():
    assert subprocess.check_output(['git','-C',repo,'rev-parse','HEAD'],text=True).strip()==v['head']
    assert not subprocess.check_output(['git','-C',repo,'status','--porcelain'],text=True).strip()
status=json.loads((RECEIVER/'transport_status.json').read_text())
for key in ['imu','beliefs','published_poses','max_queue','queued']: assert status[key]==0
assert (RECEIVER/'events.csv').stat().st_size==(RECEIVER/'receipts.jsonl').stat().st_size==0
for path in [SOURCE/'complete.marker',SOURCE/'rosbag.log',SOURCE/'effective_mode_check.json',
             SOURCE/'e1b_profile.jsonl',CONTROL/'complete.json',ROOT/'profile40_on_numerical_repeat']:
    assert not path.exists(),path
text=(SOURCE/'estimators.log').read_text()
assert 'glim_rosnode: malloc.c:4036: _int_malloc: Assertion' in text
assert 'pid 866, exit code -6' in text
assert (CONTROL/'failed.txt').exists()
processes={}
for container in ['cbsms_ws_gpu','ubuntu-jammy-humble-nvidia']:
    p=subprocess.check_output(['docker','exec',container,'ps','-eo','pid,ppid,pgid,comm'],text=True)
    for line in p.splitlines()[1:]:
        assert line.split()[-1] in ['tail','zsh','ps'],(container,line)
    processes[container]=p
# Record only scoped launch environment keys, never copy a full process env.
roslog,=SOURCE.glob('ros_logs/*/roslaunch-*.log')
envs={}
for line in roslog.read_text().splitlines():
    if ': env[{' not in line: continue
    name=line.split('process[',1)[1].split(']',1)[0]
    env=ast.literal_eval(line.split(': env[',1)[1].rstrip().removesuffix(']'))
    envs[name]={k:env.get(k) for k in ['LD_LIBRARY_PATH','CBS_E1B_PROFILE_PATH',
        'KIMERA_IMU_SAMPLE_SUPPORT','CBSMS_CBS_PROTOCOL','CBSMS_CBS_LIVE_REVISION_ENABLE']}
g=envs['glim_rosnode-1']
kimera_name,=[name for name in envs if name.endswith('kimera_vio_ros_node-4')]
k=envs[kimera_name]
assert g['CBS_E1B_PROFILE_PATH'].endswith('/profile40_on_source/e1b_profile.jsonl')
assert g['LD_LIBRARY_PATH'].startswith('/workspace/cbs_gtsam4.3/runs/e1b_local_profile_20261012/release_2:')
assert not k['CBS_E1B_PROFILE_PATH'] and 'e1b_local_profile_20261012' not in k['LD_LIBRARY_PATH']
assert g['CBSMS_CBS_PROTOCOL']==k['CBSMS_CBS_PROTOCOL']=='classic'
assert g['CBSMS_CBS_LIVE_REVISION_ENABLE']==k['CBSMS_CBS_LIVE_REVISION_ENABLE']=='false'
small=[RUN/'PROTOCOL.txt',*RUN.glob('*.py'),*ROOT.glob('*.py'),*ROOT.glob('*.sh'),
       *ROOT.glob('*.launch'),ROOT/'source.env',ROOT/'profile40_on.json',ROOT/'protected_manifest.json',
       *ROOT.joinpath('preflight').glob('*'),*CONTROL.glob('*'),
       SOURCE/'estimators.log',RECEIVER/'transport_status.json',RECEIVER/'failed.txt',
       RECEIVER/'transport_failed.txt',RECEIVER/'transport.log',RECEIVER/'receiver.log',
       RECEIVER/'resolved_config.json']
for p in SOURCE.glob('ros_logs/*/*stdout.log'): small.append(p)
copied={}
for p in small:
    if not p.is_file(): continue
    assert p.stat().st_size<1024*1024,p
    dest=OUT/p.relative_to(RUN)
    dest.parent.mkdir(parents=True,exist_ok=True)
    assert not dest.exists()
    shutil.copy2(p,dest); copied[str(dest.relative_to(HERE))]=sha(dest)
raw={str(p):dict(bytes=p.stat().st_size,sha256=sha(p)) for p in [roslog,SOURCE/'estimators.log']}
result=dict(status='BLOCKED_PRE_PLAYBACK_STARTUP_CRASH',capture_attempts=1,
    error='GLIM malloc.c:4036 _int_malloc assertion; SIGABRT exit -6 before readiness',
    preflight_passed=True,regression_tests_passed=18,protected_files_unchanged=len(manifest),
    pinned_source_repos_unchanged=previous['source_git_unchanged'],
    transport=status,launch_environment=envs,post_cleanup_processes=processes,
    actual_runtime_library_maps_verified=False,profile_rows=0,estimator_measurements=0,
    journal_repeat_executed=False,raw_log_provenance=raw,archived_evidence=copied,
    root_cause='Not established; startup allocation assertion is not a diagnosis of its corrupting write.')
with (HERE/'validation.json').open('x') as f: json.dump(result,f,indent=2)
print(result['status'],len(manifest),'protected hashes unchanged; no playback or retry')
