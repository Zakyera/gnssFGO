"""Pin explanatory sources; explicitly distinguish historical provenance."""
from pathlib import Path
import json,hashlib
H=Path(__file__).resolve().parent;W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
P=W/'runs/eig2_glim_first_edge_cpv54_20261030/prepared'
protected=json.loads((P/'protected_manifest.json').read_text())
rels=['runs/local_revision_covariance_cpi1_20261019/cbs_bridge.cpp','runs/eig2_hesai_insertion_cpv51_20261030/VioBackend.cpp','runs/e1b_local_profile_20261012/overlay/odometry_estimation_imu.cpp','src/glim_ros1/src/glim_ros/glim_ros.cpp','src/glim/src/glim/odometry/odometry_estimation_gpu.cpp','src/Kimera-VIO-ROS/src/KimeraVioRos.cpp','src/Kimera-VIO-ROS/src/CbsRelativeFrameConversion.cpp','src/Kimera-VIO-ROS/include/kimera_vio_ros/CbsRelativeFrameConversion.h','src/Kimera-VIO/src/frontend/CameraParams.cpp','src/Kimera-VIO/src/frontend/StereoCamera.cpp','src/Kimera-VIO/src/frontend/StereoVisionImuFrontend.cpp','src/Kimera-VIO/src/pipeline/StereoImuPipeline.cpp','src/Kimera-VIO/src/imu-frontend/ImuFrontend.cpp','src/Kimera-VIO/include/kimera-vio/imu-frontend/CbsNativeImuMeanBridge.h','src/cbs/src/bpsam/bpsam.cpp']
rows=[];pins={}
for rel in rels:
 p=W/rel;sha=hashlib.sha256(p.read_bytes()).hexdigest();pins[str(p)]=sha;old=protected.get(str(p));rows.append(dict(path=str(p),sha256=sha,historical_protected_match=(sha==old) if old else None))
maps={}
for mode in ['off','on']:
 p=P/f'full_{mode}_source/effective_mode_check.json';pins[str(p)]=hashlib.sha256(p.read_bytes()).hexdigest();d=json.loads(p.read_text());maps[mode]={}
 for name in ['kimera','glim']:
  paths=sorted(set(line.split()[-1] for line in d[name+'_maps'].splitlines() if any(s in line for s in ['libglim.so','libglim_ros.so','libkimera_vio.so','libkimera_vio_ros.so','libopencv_core.so'])))
  maps[mode][name]=paths
assert all(r['historical_protected_match'] is not False for r in rows)
with (H/'source_audit.json').open('x') as f:json.dump(dict(passed=True,sources=rows,runtime_library_paths=maps,input_sha256=pins,limits=['Null historical match means current explanatory source, not proven historical source-to-binary equivalence.','Actual saved endpoint/payload checks are separate evidence; shared-body conversion branch is identity.','GLIM contains OpenCV3 and4; no causal connection to checked Eigen/GTSAM mean chain established.']),f,indent=2)
print('source provenance',len(rows),'files;',sum(r['historical_protected_match'] is True for r in rows),'historically pinned')
