"""Prepare a fresh observer-only overlay; never modify baseline inputs."""
import ast, hashlib, json, shutil, subprocess, xml.etree.ElementTree as ET
from pathlib import Path
H=Path(__file__).resolve().parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
S=W/'runs/gnssfgo_eig2_velodyne_full_cpv4_20261024/prepared'
K=W/'runs/kimera_incremental_prior_cpk9_20261019'
P=W/'runs/eig2_counterfactual_cpv36_20261029'
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
assert not P.exists()
assert shutil.disk_usage(W).free>8*1024**3
pins=json.loads((S/'protected_manifest.json').read_text())
for p,s in pins.items():assert sha(p)==s,p
for p in (S/'full_off_source').rglob('*'):
    if p.is_file() and (p.name in ('objective.csv','estimators.log','kimera_odometry.csv','effective_mode_check.json') or 'kimera_profile' in p.parts):pins[str(p)]=sha(p)
P.mkdir()
shutil.copytree(K/'include',P/'include')
shutil.copytree(K/'private_build',P/'private_build',symlinks=True)
shutil.copytree(S/'full_off_source/kimera_profile',P/'kimera_profile')
for name in ('VioBackend.cpp','RetirementAudit.h','LinearParityAudit.h','OriginalChartPrior.h','CurrentChartFallback.h','NativePriorRefresh.h'):
    shutil.copy2(K/name,P/name);pins[str(K/name)]=sha(K/name)
for name in ('grandtour_eth1_kimera_stereo.launch','kimera_vio_ros_newer_college_2021_stereo.launch','kimera_vio_ros.launch'):
    shutil.copy2(S/name,P/name)
tree=ET.parse(S/'active_window.launch');root=tree.getroot();removed=[]
for child in list(root):
    if child.tag=='include' and any(s in child.get('file','') for s in ('glim_profile.launch','newer_college_cbs_ekf.launch')):
        removed.append(child.attrib);root.remove(child)
    elif child.tag=='param' and child.get('name','').startswith('/glim_rosnode/'):root.remove(child)
assert len(removed)==2
tree.write(P/'kimera_only.launch',encoding='unicode')
shutil.copy2(S/'full_off_source/effective_mode_check.json',P/'reference_effective.json')
shutil.copy2(H.parent/'kimera_only_full_rate050_cpk21_20261021/observe.py',P/'observe.py')
for name in ('Checkpoint.h','Binary.h','Counterfactual.cpp','observer_test.cpp'):shutil.copy2(H/name,P/name)
old='/workspace/cbs_gtsam4.3/runs/kimera_incremental_prior_cpk9_20261019'
new='/workspace/cbs_gtsam4.3/runs/eig2_counterfactual_cpv36_20261029'
for name in ('build_commands.json','actual_test_commands.json'):
    d=json.loads((K/name).read_text())
    d=json.loads(json.dumps(d).replace(old,new))
    if name=='actual_test_commands.json':
        d=json.loads(json.dumps(d).replace(new+'/test.cpp',new+'/observer_test.cpp'))
    (P/name).write_text(json.dumps(d,indent=2)+'\n')
for p in P.rglob('*.launch'):ET.parse(p)
(H/'protected_before.json').write_text(json.dumps(pins,indent=2)+'\n')
(H/'preparation.json').write_text(json.dumps(dict(root=str(P),removed=removed,protected=len(pins),stage='overlay copied, hook/build/run pending'),indent=2)+'\n')
print('Prepared',P,'protected',len(pins))

# Private header-only accessors: no layout, virtual dispatch or native policy change.
gsource=K/'include/gtsam_unstable/slam/SmartStereoProjectionPoseFactor.h'
gdest=P/'include/gtsam_unstable/slam/SmartStereoProjectionPoseFactor.h';gdest.parent.mkdir(parents=True,exist_ok=True)
text=gsource.read_text();needle='  EIGEN_MAKE_ALIGNED_OPERATOR_NEW'
access='''  const SmartStereoProjectionParams& cpv36Params() const { return params_; }
  double cpv36Sigma() const { return noiseModel_->sigma(); }
  const std::vector<Pose3>& cpv36CachedPoses() const { return cameraPosesTriangulation_; }
  void cpv36RestoreCache(const TriangulationResult& r,const std::vector<Pose3>& p) { result_=r;cameraPosesTriangulation_=p; }
'''
assert text.count(needle)==1;gdest.write_text(text.replace(needle,needle+'\n'+access))
pins[str(gsource)]=sha(gsource)
for q in (H.parent/'eig2_kimera_off_cpv35_20261029').iterdir():
    if q.is_file():pins[str(q)]=sha(q)
for q in (W/'runs/eig2_kimera_off_cpv35_20261029').rglob('*'):
    if q.is_file() and not q.is_symlink():pins[str(q)]=sha(q)
(H/'protected_before.json').write_text(json.dumps(pins,indent=2)+'\n')
p=P/'include/cbs/bpsam/incremental_fixed_lag_bpsam_smoother.h';text=p.read_text()
text='#include "Checkpoint.h"\n'+text
needle='    if (fl4q::enabled()) fl4q::accept(current_timestamp,isam_,fl4m_before,constrainedKeys);'
assert text.count(needle)==1
text=text.replace(needle,needle+'\n    cpv36::capture(current_timestamp,isam_,fl4m_before);')
p.write_text(text)
