#!/usr/bin/env python3
"""Derive one isolated source-only diagnostic from the frozen ETH1 harness."""
import hashlib
import json
import shutil
from pathlib import Path

HERE=Path(__file__).resolve().parent
WS=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
BASE=WS/'runs/gnssfgo_eth1_vw_jacobian_live160_20261011/prepared'
ROOT=WS/'runs/glim_e1_real_graph_20261011/prepared'
PRIOR=WS/'runs/glim_sync_compatible_batching_e1_20261011/private'
ROOT.mkdir(parents=True,exist_ok=False)
(ROOT/'private_build').mkdir(); (ROOT/'audit').mkdir()

def inside(p): return '/workspace/'+str(p.relative_to('/home/yeranis/repos/V4RL'))
def edit(text,a,b):
    assert text.count(a)==1,(a,text.count(a))
    return text.replace(a,b)
def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()

for name in ['active_window.launch','grandtour_eth1_kimera_stereo.launch','kimera_vio_ros.launch',
             'kimera_vio_ros_newer_college_2021_stereo.launch','verify_effective.py']:
    shutil.copy2(BASE/name,ROOT/name)
source=(BASE/'source_runner.sh').read_text()
source=edit(source,'"${duration_sec}" == 160.0','"${duration_sec}" == 20.0')
source=edit(source,'Prepared 160-second ON diagnostic-budget validation only.','Prepared E1 real-graph 20-second diagnostic only.')
source=edit(source,'min_glim_rows=1400\nmin_kimera_rows=600','min_glim_rows=170\nmin_kimera_rows=70')
source=edit(source,'"${run_root}/${run_name%_source}_receiver/events.csv"','"${run_dir}/recordings/cbs.bag"')
source=edit(source,'/kimera/cbs/odom_belief_in /kimera/cbs/odom_belief_out \\\n',
                   '/boxi/cpt7/imu /kimera/cbs/odom_belief_in /kimera/cbs/odom_belief_out \\\n')
source=source.replace('experiment=gnssfgo_eth1_vw_jacobian_live160_20261011','experiment=glim_e1_real_graph_20261011')
(ROOT/'source_runner.sh').write_text(source)
env=(BASE/'source.env').read_text().replace(inside(BASE),inside(ROOT)).replace('11399','11401').replace(':-160.0',':-20.0')
env=edit(env,'CBSMS_USE_EXISTING_ROS_MASTER=true','CBSMS_USE_EXISTING_ROS_MASTER=false')
env+='\nexport GLIM_E1_REAL_AUDIT_ROOT='+inside(ROOT/'audit')+'\n'
(ROOT/'source.env').write_text(env)
verify=(ROOT/'verify_effective.py').read_text().replace("== '160.0'","== '20.0'")
verify=edit(verify,"with (source/'effective_mode_check.json').open('x') as f:",
  "assert glim.get('glim_cbs_experimental_local_gpu_batching',False) is False\n"
  "assert str(root/'private_build/libgtsam_points_cuda.so.1.2.1') in glim_loaded\n"
  "assert str(root/'private_build/libcbs.so') in glim_loaded\n"
  "with (source/'effective_mode_check.json').open('x') as f:")
(ROOT/'verify_effective.py').write_text(verify)
coverage=(BASE/'coverage_checks.py').read_text()
start=coverage.index('    imu=[]\n'); end=coverage.index('    assert len(imu)>100',start)
coverage=coverage[:start]+'''    import rosbag
    with rosbag.Bag(str(journal)) as bag:
        imu=[m.header.stamp.to_sec() for _,m,_ in bag.read_messages(topics=['/boxi/cpt7/imu'])]
'''+coverage[end:]
(ROOT/'coverage_checks.py').write_text(coverage)

# Only diagnostic instrumentation is in this private source copy. The original
# source, production parameter, and previous candidate libraries are untouched.
cpp=(WS/'src/glim_ros1/src/glim_ros/cbs_bridge.cpp').read_text()
cpp='#include "RealGraphAudit.h"\n'+cpp
old='''      if (experimental_local_gpu_batching_) {
        sidecar_->setMarginalizationGraph('''
new='''      const size_t e1_sample=e1_real::selectedCall();
      if (e1_sample) {
        sidecar_->setMarginalizationGraph(cbs::BPSAM::MarginalizationType::LOCAL,
          [&](const gtsam::NonlinearFactorGraph& graph,const gtsam::Values& values) {
            std::vector<e1_real::Pair> pairs;
            for(const auto& p:explicit_pairs) pairs.push_back({p.first,p.second,
              smoother.calculateEstimate<gtsam::Pose3>(p.first),
              smoother.calculateEstimate<gtsam::Pose3>(p.second)});
            auto linear=e1_real::compare(e1_sample,stamp.toSec(),graph,values,pairs);
            for(const auto& p:pairs) {
              if(!p.pa.equals(smoother.calculateEstimate<gtsam::Pose3>(p.a),0) ||
                 !p.pb.equals(smoother.calculateEstimate<gtsam::Pose3>(p.b),0)) {
                std::ofstream(std::string(std::getenv("GLIM_E1_REAL_AUDIT_ROOT"))+"/FAILED.txt") << "main means changed";
                throw std::runtime_error("E1 main means changed");
              }
            }
            std::ofstream(std::string(std::getenv("GLIM_E1_REAL_AUDIT_ROOT"))+"/sample_"+
                          std::to_string(e1_sample)+"/means_unchanged.marker") << "exact main endpoint poses preserved\\n";
            return linear;
          });
      } else if (experimental_local_gpu_batching_) {
        sidecar_->setMarginalizationGraph('''
cpp=edit(cpp,old,new)
(ROOT/'private_build/cbs_bridge.cpp').write_text(cpp)
shutil.copy2(HERE/'RealGraphAudit.h',ROOT/'private_build/RealGraphAudit.h')
# Copy small runtime DSOs into the new private root, avoiding dangling host links.
for name,path in [('libcbs.so',WS/'runs/glim_local_gpu_batching_e1_20261011/private/libcbs.so'),
                  ('libgtsam_points_cuda.so.1.2.1',PRIOR/'libgtsam_points_cuda.so.1.2.1')]:
    shutil.copy2(path,ROOT/'private_build'/name)
for name in ['libgtsam_points_cuda.so','libgtsam_points_cuda.so.1']:
    (ROOT/'private_build'/name).symlink_to('libgtsam_points_cuda.so.1.2.1')
commands=json.loads((PRIOR/'wrapper_attempt1_commands.json').read_text())
compile_cmd=commands[1]
compile_cmd=[inside(ROOT/'private_build/cbs_bridge.cpp') if x.endswith('/src/glim_ros1/src/glim_ros/cbs_bridge.cpp') else
             inside(ROOT/'private_build/cbs_bridge.o') if x==inside(PRIOR/'cbs_bridge.o') else x for x in compile_cmd]
link=commands[-1]
link=[inside(ROOT/'private_build/libglim_ros.so') if x==inside(PRIOR/'libglim_ros.so') else
      inside(ROOT/'private_build/cbs_bridge.o') if x==inside(PRIOR/'cbs_bridge.o') else x for x in link]
(ROOT/'build_commands.json').write_text(json.dumps([compile_cmd,link],indent=2))
shutil.copy2(HERE/'run.py',ROOT/'run.py')
shutil.copy2(HERE/'build.py',ROOT/'build.py')
manifest={str(p):sha(p) for p in ROOT.rglob('*') if p.is_file()}
manifest['baseline_source_runner_sha256']=sha(BASE/'source_runner.sh')
(ROOT/'prepared_manifest.json').write_text(json.dumps(manifest,indent=2))
print(ROOT)
