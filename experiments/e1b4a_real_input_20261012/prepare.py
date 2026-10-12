"""Derive a fresh source-only diagnostic; never edit protected artifacts."""
from pathlib import Path
import hashlib
import json
import shutil

HERE=Path(__file__).resolve().parent
WS=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
ROOT=WS/'runs/e1b4a_real_input_20261012'
BASE=WS/'runs/glim_e1_real_graph_20261011/prepared'
PROFILE=WS/'runs/gnssfgo_e1b_profile40_mapguard_20261012/prepared'
BATCH=WS/'runs/e1b4a_local_update_batching_20261012'
CHECK=WS/'runs/e1b4a_snapshot_checker_20261012'
def inside(p):return str(p).replace(str(WS),'/workspace/cbs_gtsam4.3')
def sha(p):
    p=Path(str(p).replace('/workspace/cbs_gtsam4.3',str(WS)))
    if p.is_symlink():return sha(p.readlink() if p.readlink().is_absolute() else p.parent/p.readlink())
    return hashlib.sha256(p.read_bytes()).hexdigest()
def replace(s,a,b):
    assert s.count(a)==1,(a,s.count(a))
    return s.replace(a,b)
def main():
    ROOT.mkdir(exist_ok=False)
    for d in ['authoring','audit','private_build']: (ROOT/d).mkdir()
    for name in ['ExactGpuSnapshot.h','ExactGpuSnapshot.cpp','LocalUpdateCheck.h']:
        shutil.copy2(CHECK/'authoring'/name,ROOT/'authoring'/name)
    shutil.copytree(CHECK/'include',ROOT/'include')
    shutil.copy2(HERE/'RealInputObserver.h',ROOT/'authoring/RealInputObserver.h')
    cpp=(BATCH/'overlay/cbs_bridge.cpp').read_text()
    cpp='#include "RealInputObserver.h"\n'+cpp
    old='''    gtsam::e1b4a::ScopedLinearizer batch_scope(
        glim::e1b4a::enabled() ? &glim::e1b4a::linearize : nullptr);
    sidecar_->update(new_factors, new_values, new_stamps);'''
    cpp=replace(cpp,old,'    e1b4real::update(*sidecar_, new_factors, new_values, new_stamps);')
    (ROOT/'authoring/cbs_bridge.cpp').write_text(cpp)
    for name in ['source_runner.sh','source.env','coverage_checks.py']:
        s=(BASE/name).read_text().replace(inside(BASE),inside(ROOT))
        s=s.replace('glim_e1_real_graph_20261011','e1b4a_real_input_20261012')
        s=s.replace('11401','11413')
        s=s.replace('export GLIM_E1_REAL_AUDIT_ROOT=', '# unused prior observer: ')
        (ROOT/name).write_text(s)
    for name in ['active_window.launch','glim_profile.launch','grandtour_eth1_kimera_stereo.launch',
                 'kimera_vio_ros.launch','kimera_vio_ros_newer_college_2021_stereo.launch','map_guard.py']:
        s=(PROFILE/name).read_text().replace(inside(PROFILE),inside(ROOT))
        if name=='glim_profile.launch':
            s=s.replace('/workspace/cbs_gtsam4.3/runs/e1b_startup_fix_20261012/fixed_release',inside(ROOT/'release_1'))
            s=replace(s,'<env name="CBS_E1B_PROFILE_PATH" value="'+inside(ROOT)+'/profile40_on_source/e1b_profile.jsonl"/>',
              '<env name="CBS_E1B_REAL_INPUT_ROOT" value="'+inside(ROOT/'audit')+'"/>\n'
              '\t\t<env name="CBS_E1B_LOCAL_UPDATE_GPU_BATCHING" value="0"/>')
        (ROOT/name).write_text(s)
    # Kimera keeps its historical, non-profiler CBS library path. GLIM gets the
    # distinct diagnostic DSO chain only in its own launch environment.
    for name in ['libcbs.so','libgtsam_points_cuda.so.1.2.1']:
        shutil.copy2(BASE/'private_build'/name,ROOT/'private_build'/name)
    for name in ['libgtsam_points_cuda.so','libgtsam_points_cuda.so.1']:
        (ROOT/'private_build'/name).symlink_to('libgtsam_points_cuda.so.1.2.1')
    s=(PROFILE/'verify_effective.py').read_text().replace("== '40.0'","== '20.0'")
    s=s.replace('/workspace/cbs_gtsam4.3/runs/e1b_startup_fix_20261012/fixed_release',inside(ROOT/'release_1'))
    s=replace(s,"assert str(candidate) not in loaded and 'libe1b_profile' not in loaded",
                "assert str(candidate) not in loaded and 'libe1b_profile' not in loaded")
    s=replace(s,"assert env(glim_pid)['CBS_E1B_PROFILE_PATH'] == str(source/'e1b_profile.jsonl')",
      "assert env(glim_pid)['CBS_E1B_REAL_INPUT_ROOT'] == str(root/'audit')\n"
      "assert env(glim_pid)['CBS_E1B_LOCAL_UPDATE_GPU_BATCHING'] == '0'\n"
      "assert 'CBS_E1B_PROFILE_PATH' not in env(glim_pid)")
    (ROOT/'verify_effective.py').write_text(s)
    s=(HERE.parent/'e1_real_graph_20261011/run.py').read_text()
    s=s.replace('glim_e1_real_graph_20261011/prepared','e1b4a_real_input_20261012')
    s=replace(s,"        d=root/'audit'/('sample_'+str(n))\n        assert json.loads((d/'summary.json').read_text())['passed']\n        assert (d/'means_unchanged.marker').exists()",
       "        assert json.loads((root/'audit'/('sample_'+str(n)+'.json')).read_text())['passed']")
    s=s.replace('STARTED E1 real-graph','STARTED E1b.4a exact-boundary diagnostic')
    (ROOT/'run.py').write_text(s)
    shutil.copy2(HERE/'build.py',ROOT/'build.py')
    # Frozen checker and original baseline protection sets, plus current private
    # source/harness inputs. This task adds outputs, it must not mutate inputs.
    protected=json.loads((PROFILE/'protected_manifest.json').read_text())
    protected.update(json.loads((CHECK/'protected.json').read_text()))
    for directory in [BASE,PROFILE,BATCH/'release_2',CHECK/'authoring',CHECK/'include']:
        for p in directory.rglob('*'):
            if p.is_file() and (p.suffix in ['.py','.sh','.env','.launch','.h','.cpp','.so','.json']
                                or '.so.' in p.name):protected[str(p)]=sha(p)
    (ROOT/'protected.json').write_text(json.dumps(protected,indent=2))
    shutil.copy2(CHECK/'source_git.json',ROOT/'source_git.json')
    (ROOT/'prepared_manifest.json').write_text(json.dumps({str(p):sha(p) for p in ROOT.rglob('*') if p.is_file()},indent=2))
    print(ROOT)
if __name__=='__main__':main()
