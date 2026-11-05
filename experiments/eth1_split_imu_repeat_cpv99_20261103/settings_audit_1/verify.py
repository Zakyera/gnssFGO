"""Seal supplemental settings audit; read-only against all experiment evidence."""
import difflib, hashlib, json, subprocess
from pathlib import Path
O=Path(__file__).resolve().parent
E=O.parent.parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
names={'91':'eth1_split_imu_persistent_on_cpv91_20261102_attempt1',
       '95':'eth1_split_imu_live_pair_cpv95_20261103_attempt3',
       '99':'eth1_split_imu_repeat_cpv99_20261103'}
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
a=json.loads((O/'audit.json').read_text());pins=dict(a['input_sha256'])
terminal={}
for label,root in [('91',E/names['91']),('95',E/'eth1_split_imu_live_pair_cpv95_20261103_continuation1')]:
    z=json.loads((root/'complete.json').read_text());assert z['passed']
    for path,d in z['output_sha256'].items():assert sha(path)==d,path
    terminal[label]=len(z['output_sha256'])
used={}
for name in ['experiment_runner_used.sh','grandtour_launch_used.launch','newer_college_reference_launch.launch']:
    texts={}
    for run,stem in names.items():
        p=W/'runs'/stem/'prepared'/('on_source' if run=='91' else 'full_on_source')/name
        pins[str(p)]=sha(p)
        texts[run]=p.read_text().replace(stem,'<RUN>').replace('full_on_source','<SOURCE>').replace('on_source','<SOURCE>')
    used[name]={k:list(difflib.unified_diff(texts[k].splitlines(),texts['99'].splitlines(),lineterm='')) for k in ['91','95']}
lib=W/'runs'/names['95']/'prepared/on/private_build/libkimera_vio.so'
assert 'CPK9 promotion changes cached prior' in subprocess.check_output(['strings',str(lib)],text=True)
dependencies={}
for name in ['libgtsam.so.4.3a1','libgtsam_unstable.so.4.3a1']:
    p=W/'devel/lib'/name;s=p.stat()
    dependencies[str(p)]=dict(sha256=sha(p),inode=s.st_ino,mtime_ns=s.st_mtime_ns,ctime_ns=s.st_ctime_ns,
        qualification='Historical maps have this same path/inode; mtime and ctime predate CP91. No historical content hash in selected manifests.')
for path,d in pins.items():assert sha(path)==d,path
artifacts={str(p):sha(p) for p in [O/'audit.py',O/'audit.json',O/'verify.py',O/'REPORT.txt']}
out=dict(passed=True,experiment_success=False,previous_terminal_hashes_verified=terminal,
    audit_input_hashes_verified=len(pins),executed_artifact_differences=used,
    successful_cp95_binary_contains_same_guard=True,shared_gtsam_metadata=dependencies,
    no_capture_build_configuration_change_or_scoring=True,input_sha256=pins,artifact_sha256=artifacts)
with (O/'verification.json').open('x') as f:json.dump(out,f,indent=2)
print(json.dumps({k:v for k,v in out.items() if k not in ('input_sha256','artifact_sha256','executed_artifact_differences','shared_gtsam_metadata')},indent=2))
