#!/usr/bin/env python3
"""Mechanical private-harness derivation; never edits an estimator or old run."""
from pathlib import Path
import hashlib
import json
import shutil

HERE=Path(__file__).resolve().parent
WS=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
BASE=WS/'runs/gnssfgo_eth1_vw_jacobian_live160_20261011'
RUN=WS/'runs/gnssfgo_e1_live60_20261011'
ROOT=RUN/'prepared'
PRIOR=WS/'runs/glim_sync_compatible_batching_e1_20261011/private'
ROOT.mkdir(parents=True,exist_ok=False)
(ROOT/'private_build').mkdir()
def put(p,s):
    with p.open('x') as f:f.write(s)
def replace(s,a,b):
    assert s.count(a)==1,(a,s.count(a))
    return s.replace(a,b)
def inside(p):return '/workspace/'+str(p.relative_to('/home/yeranis/repos/V4RL'))
for name in ['postvalidate.py','check_lifecycle.py','validate_diagnostics.py','status.py']:
    s=(BASE/name).read_text().replace('full160','live60').replace('160-SECOND','60-SECOND')
    put(RUN/name,s)
for name in ['experiment.py','run_case.py','source.env','source_runner.sh','coverage_checks.py',
             'verify_effective.py','validate_native_bidirectional.py','validate_numerical_case.py',
             'numerical_equivalence.py','active_window.launch','grandtour_eth1_kimera_stereo.launch',
             'kimera_vio_ros.launch','kimera_vio_ros_newer_college_2021_stereo.launch']:
    s=(BASE/'prepared'/name).read_text()
    s=s.replace('gnssfgo_eth1_vw_jacobian_live160_20261011','gnssfgo_e1_live60_20261011')
    s=s.replace('11399','11403').replace('full160','live60').replace('160.0','60.0')
    if name=='source.env':s=s.replace(':-160',':-60')
    if name=='source_runner.sh':
        s=s.replace('Prepared 160-second ON diagnostic-budget validation only.','Prepared 60-second ON E1 live acceptance only.')
        s=replace(s,'min_glim_rows=1400\nmin_kimera_rows=600','min_glim_rows=550\nmin_kimera_rows=250')
    if name=='run_case.py':
        # The saved OFF is merely an unchanged protection precondition, not rerun.
        s=s.replace('gnssfgo_eth1_covariance160_20261010/prepared/live60_off_validation.json',
                    'gnssfgo_eth1_covariance160_20261010/prepared/full160_off_validation.json')
        s=replace(s,'duration=160','duration=60')
        s=replace(s,"                    samples.write(json.dumps(status)+'\\n')",
          "                    samples.write(json.dumps(status)+'\\n')\n"
          "                    assert status['queued']<1000 and status['max_queue']<1000, 'live queue gate exceeded'")
    if name=='active_window.launch':
        anchor='  <param name="/glim_rosnode/glim_cbs_experimental_native_imu_mean_alignment" value="true"/>'
        s=replace(s,anchor,anchor+'\n  <param name="/glim_rosnode/glim_cbs_experimental_local_gpu_batching" value="true"/>')
    if name=='verify_effective.py':
        s=replace(s,"expected = dict(glim_cbs_mode=","expected = dict(glim_cbs_experimental_local_gpu_batching=True, glim_cbs_mode=")
        s=replace(s,"with (source/'effective_mode_check.json').open('x') as f:",
          "assert str(root/'private_build/libgtsam_points_cuda.so.1.2.1') in glim_loaded\n"
          "assert str(root/'private_build/libcbs.so') in glim_loaded\n"
          "with (source/'effective_mode_check.json').open('x') as f:")
    if name=='validate_numerical_case.py':
        s=replace(s,'),160,imu_times[0]', '),60,imu_times[0]')
        anchor="            if key.startswith('glim_cbs_active_factor_diagnostics_'):"
        s=replace(s,anchor,"            if key=='glim_cbs_experimental_local_gpu_batching':\n"
          "                assert old is None and new is True\n                continue\n"+anchor)
        s=replace(s,"    assert transport['queued']==0", "    assert transport['queued']==0 and transport['max_queue']<1000")
    put(ROOT/name,s)
cfg=json.loads((BASE/'prepared/full160_on.json').read_text())
cfg['sequence']='ETH1 first60s; E1 private LOCAL GPU batching live acceptance'
cfg['ros_master_uri']='http://127.0.0.1:11403'
for source in cfg['sources']:source['session']='eth1_e1_live60_on_20261011'
put(ROOT/'live60_on.json',json.dumps(cfg,indent=2)+'\n')
for name,src in [('libglim_ros.so',PRIOR/'libglim_ros.so'),
                 ('libgtsam_points_cuda.so.1.2.1',PRIOR/'libgtsam_points_cuda.so.1.2.1'),
                 ('libcbs.so',WS/'runs/glim_local_gpu_batching_e1_20261011/private/libcbs.so')]:
    shutil.copy2(src,ROOT/'private_build'/name)
for name in ['libgtsam_points_cuda.so','libgtsam_points_cuda.so.1']:
    (ROOT/'private_build'/name).symlink_to('libgtsam_points_cuda.so.1.2.1')
shutil.copy2(HERE/'PROTOCOL.txt',RUN/'PROTOCOL.txt')
print(ROOT)
