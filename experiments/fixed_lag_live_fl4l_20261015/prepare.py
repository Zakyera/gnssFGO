"""Generate an isolated diagnostic overlay; never overwrite production files."""
from pathlib import Path
import difflib, hashlib, json, shutil, shlex
HERE=Path(__file__).resolve().parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
ROOT=W/'runs/fixed_lag_same_graph_fl4l_20261015'
FROZEN=W/'runs/gnssfgo_eth1_fixed_lag_live120_fl4g_20261015/prepared'
def container(p): return str(p).replace('/home/yeranis/repos/V4RL','/workspace')
def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()
def replace(text,a,b):
    assert text.count(a)==1,(a,text.count(a))
    return text.replace(a,b)
ROOT.mkdir()
manifest=json.loads((FROZEN/'protected_manifest.json').read_text())
assert all(Path(p).is_file() and sha(Path(p))==v for p,v in manifest.items())
(ROOT/'protected_before.json').write_text(json.dumps(manifest,indent=2))
for name in ['OriginalChartPrior.h','CaptureHook.inc','test.cpp']:
    shutil.copy2(HERE/name,ROOT/name)
shutil.copy2(HERE/'CMAKE.txt',ROOT/'CMakeLists.txt')
helper=ROOT/'include/cbs/bpsam/fixed_lag_objective_audit.h'
helper.parent.mkdir(parents=True)
shutil.copy2(W/'src/cbs/include/cbs/bpsam/fixed_lag_objective_audit.h',helper)
original=W/'src/Kimera-VIO/src/backend/VioBackend.cpp'
s=original.read_text()
s=replace(s,'#include "kimera-vio/backend/VioBackend.h"',
          '#include "kimera-vio/backend/VioBackend.h"\n#include "OriginalChartPrior.h"')
s=replace(s,'((1082u <= update && update <= 1085u) ||\n          (1091u <= update && update <= 1094u))','update == 438u')
s=replace(s,'void saveBadUpdateAudit(const FrameId update,',
          '#include "CaptureHook.inc"\n\nvoid saveBadUpdateAudit(const FrameId update,')
s=replace(s,'  gtsam::Values bad_update_values_before;',
          '  gtsam::Values bad_update_values_before;\n  gtsam::Values fl4l_theta_before;')
s=replace(s,'    bad_update_graph_before = getMainBackendFactors().clone();',
          '    fl4l_theta_before = smoother_->getLinearizationPoint();\n    bad_update_graph_before = getMainBackendFactors().clone();')
a='''          saveBadUpdateAudit(cur_id,
                             bad_update_graph_before,
                             bad_update_values_before,
                             new_factors_tmp,
                             delete_slots,
                             smoother_->badUpdateStageAudit()
                                 ? &*smoother_->badUpdateStageAudit()
                                 : nullptr,
                             getMainBackendFactors(),
                             state_);'''
b='''          captureOriginalChart438(bad_update_graph_before,
              bad_update_values_before, fl4l_theta_before,
              smoother_->badUpdateStageAudit() ? &*smoother_->badUpdateStageAudit() : nullptr,
              smoother_->getLinearizationPoint());'''
s=replace(s,a,b)
(ROOT/'VioBackend.cpp').write_text(s)
(ROOT/'VioBackend.diff').write_text(''.join(difflib.unified_diff(original.read_text().splitlines(True),s.splitlines(True),fromfile=str(original),tofile='PRIVATE/VioBackend.cpp')))
commands=json.loads((W/'build/kimera_vio/compile_commands.json').read_text())
entry=next(x for x in commands if x['file'].endswith('/backend/VioBackend.cpp'))
compile_cmd=shlex.split(entry['command'])
compile_cmd[compile_cmd.index('-o')+1]=container(ROOT/'VioBackend.cpp.o')
compile_cmd[compile_cmd.index('-c')+1]=container(ROOT/'VioBackend.cpp')
compile_cmd[1:1]=['-I'+container(ROOT),'-I'+container(ROOT/'include')]
link=shlex.split((W/'build/kimera_vio/CMakeFiles/kimera_vio.dir/link.txt').read_text())
link[link.index('-o')+1]=container(ROOT/'private_build/libkimera_vio.so')
idx=next(i for i,x in enumerate(link) if x.endswith('/backend/VioBackend.cpp.o'))
link[idx]=container(ROOT/'VioBackend.cpp.o')
(ROOT/'build_commands.json').write_text(json.dumps({'cwd':entry['directory'],'compile':compile_cmd,'link':link},indent=2))
shutil.copytree(FROZEN/'private_build',ROOT/'private_build',symlinks=True)
for name in ['source.env','source_runner_on.sh','active_window_on.launch','glim_profile_on.launch',
             'grandtour_eth1_kimera_stereo.launch','kimera_vio_ros_newer_college_2021_stereo.launch',
             'kimera_vio_ros.launch','verify_effective_on.py','map_guard.py','coverage_checks.py']:
    text=(FROZEN/name).read_text().replace(container(FROZEN),container(ROOT))
    text=text.replace('/pilot120_on_source/e1b_profile.jsonl','/on_source_capture/e1b_profile.jsonl')
    if name=='source.env':
        text=text.replace(':-120.0',':-94.0').replace('=11434','=11435').replace('CBSMS_USE_EXISTING_ROS_MASTER=true','CBSMS_USE_EXISTING_ROS_MASTER=false')
        text+='\nexport CBSMS_KIMERA_BAD_UPDATE_AUDIT_DIR="${CBSMS_RUN_ROOT}/capture"\n'
    if name=='source_runner_on.sh':
        text=text.replace('== 120.0','== 94.0').replace('120-second classic CBS-ON fixed-lag capture','94-second private source-only diagnostic capture')
        text=text.replace('min_glim_rows=1100','min_glim_rows=900').replace('min_kimera_rows=500','min_kimera_rows=440')
        text=text.replace('"${run_root}/${run_name%_source}_receiver/events.csv"','"${run_dir}/recordings/cbs.bag"')
        text=text.replace('  /glim/cbs/odometry /kimera_vio_ros/odometry \\\n','  /boxi/cpt7/imu /glim/cbs/odometry /kimera_vio_ros/odometry \\\n')
        text=text.replace('experiment=gnssfgo_eth1_fixed_lag_live120_fl4g_20261015','experiment=fixed_lag_same_graph_fl4l_20261015_source_only')
    if name=='verify_effective_on.py':
        text=text.replace("== '120.0'","== '94.0'")
        text=text.replace("assert '/workspace/cbs_gtsam4.3/build/kimera_vio/libkimera_vio.so' in loaded",
                          "assert str(root/'private_build/libkimera_vio.so') in loaded")
        text+="\nassert flags['kimera_bad_update_audit_dir'] == str(root/'capture')\n"
    if name=='coverage_checks.py':
        a="""    with journal.open() as f:
        for line in f:
            v=line.strip().split(',')
            if v[0]=='I': imu.append(int(v[2])*1e-9)"""
        b="""    import rosbag
    with rosbag.Bag(str(journal)) as bag:
        for _,message,_ in bag.read_messages(topics=['/boxi/cpt7/imu']):
            imu.append(message.header.stamp.to_nsec()*1e-9)"""
        text=replace(text,a,b)
    (ROOT/name).write_text(text)
    (ROOT/(name+'.diff')).write_text(''.join(difflib.unified_diff((FROZEN/name).read_text().splitlines(True),text.splitlines(True),fromfile=str(FROZEN/name),tofile='PRIVATE/'+name)))
(ROOT/'manifest.json').write_text(json.dumps({str(p.relative_to(ROOT)):sha(p) for p in ROOT.rglob('*') if p.is_file()},indent=2))
print(ROOT)
