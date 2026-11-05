"""Freeze a minimal isolated CBS build; never writes shared build/source files."""
import hashlib,json,shutil,subprocess
from pathlib import Path
H=Path(__file__).resolve().parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
R=W/'runs/leica2_rotation_weight_cpv6_20261024'
E=W/'runs/glim_local_gpu_batching_e1_20261011/private'
B=W/'runs/gnssfgo_leica2_velodyne_full_cpv4_20261024/prepared'
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def cp(p):return str(p).replace('/home/yeranis/repos/V4RL','/workspace')
assert not R.exists()
assert subprocess.check_output(['git','-C',str(W/'src/cbs'),'diff','7c3a1e66767938e105d46fd072bd853ad5ec50c2','--','src/bpsam/bpsam.cpp','include/cbs/bpsam/bpsam.h'])==b''
assert sha(E/'libcbs.so')==sha(B/'private_build/libcbs.so')
R.mkdir()
shutil.copytree(W/'src/cbs/include',R/'include')
shutil.copy2(W/'src/cbs/src/bpsam/bpsam.cpp',R/'bpsam.cpp')
commands=json.loads((E/'cbs_commands.json').read_text())
control=[[x.replace(cp(E/'bpsam.o'),cp(R/'control.o')).replace(cp(E/'libcbs.so'),cp(R/'control.so')) for x in cmd] for cmd in commands]
candidate=[[x.replace(cp(E/'bpsam.o'),cp(R/'candidate.o')).replace(cp(E/'libcbs.so'),cp(R/'libcbs.so')).replace(cp(W/'src/cbs/src/bpsam/bpsam.cpp'),cp(R/'bpsam.cpp')).replace('-I'+cp(W/'src/cbs/include'),'-I'+cp(R/'include')) for x in cmd] for cmd in commands]
for name,data in [('control_commands.json',control),('candidate_commands.json',candidate)]:
 with (R/name).open('x') as f:json.dump(data,f,indent=2)
pins=json.loads((B/'protected_manifest.json').read_text())
for p in [W/'src/cbs/src/bpsam/bpsam.cpp',*list((W/'src/cbs/include').rglob('*.h')),W/'build/cbs/CMakeFiles/cbs.dir/src/gbp/gaussian.cpp.o',E/'cbs_commands.json']:
 pins[str(p)]=sha(p)
with (R/'protected_before.json').open('x') as f:json.dump(pins,f,indent=2)
print(json.dumps({'root':str(R),'baseline_cbs':sha(E/'libcbs.so'),'protected':len(pins)}))
