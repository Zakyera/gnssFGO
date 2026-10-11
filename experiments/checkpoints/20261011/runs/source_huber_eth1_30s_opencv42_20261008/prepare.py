"""Pin isolated binaries/configuration before running; no estimator execution."""
from pathlib import Path
import hashlib
import json
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parent
REPO=Path('/home/yeranis/repos/gnssFGO')
OLD=ROOT.parent/'gnssfgo_eth1_live_wnoa_20261007'
ISOLATED=Path('/home/yeranis/repos/V4RL/source_huber_20261008')
BINARY=Path('/home/yeranis/results/gnssfgo_eth1_local_prediction_20261007/build/wnoa/replay')
ENV=dict(LD_LIBRARY_PATH='/home/yeranis/results/gnssfgo_synthetic_uncertainty_20261004/baseline/runtime/lib',
         OMP_NUM_THREADS='1',WNOJ_STRICT_PUBLICATION='1',WNOJ_INCREMENTAL_PROFILE='nominal')
def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()
def exclusive(p,data):
    with p.open('x') as f: json.dump(data,f,indent=2)
def main():
    startup=json.loads((ROOT/'startup_gate/validation.json').read_text())
    assert startup['passed'] and startup['data_played'] is False
    assert set(startup['cases'])=={'gaussian','huber'}
    assert '--cbs_source_loss=huber' in (ISOLATED/'logs/launch_args_huber.txt').read_text()
    assert sha(BINARY)=='fb56ffc5c6c9cb575d6129289688caeabb4e3baba32e429b857f3d6a30e50d1f'
    for name in ['integration_isolated.xml','sanitized.xml']:
        doc=ET.parse(ISOLATED/'logs'/name).getroot()
        assert int(doc.attrib['tests'])==8 and int(doc.attrib['failures'])==0 and int(doc.attrib['errors'])==0
    assert 'bpsam_belief_odom_smoke: ok' in (ISOLATED/'logs/legacy_smoke.log').read_text()
    files=[BINARY, ISOLATED/'install/lib/libcbs.so',
           ISOLATED/'build/kimera_vio_opencv42/libkimera_vio.so',ISOLATED/'devel/lib/libglim_ros.so',
           ISOLATED/'devel/lib/glim_ros/glim_rosnode']
    base=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
    files += [base/'devel/lib'/n for n in ['libcbs.so','libkimera_vio.so','libglim_ros.so','libgtsam.so.4.3a1']]
    files += [REPO/'tools'/n for n in ['live_receiver_protocol.py','grandtour_live_receiver_ros1.py','launch_grandtour_live_receiver.py']]
    profile=base/'runs/grandtour_eth1_presentation/grandtour_eth1_stereo_shared_cpt7_no_sidecar_cbs_off_full_346s_20260907'
    for name in ['glim_config','kimera_profile']:
        files+=sorted(p for p in (profile/name).rglob('*') if p.is_file())
    for case in ['off_gaussian','on_gaussian','on_huber']:
        config=json.loads((OLD/'smoke_on.json').read_text())
        config['sequence']='ETH1 first30s; source Huber engineering: '+case
        config['ros_master_uri']='http://127.0.0.1:11389'
        for source in config['sources']:
            source['session']='eth1_source_huber_opencv42_'+case+'_20261008'
        if (ROOT/(case+'.json')).exists():
            assert json.loads((ROOT/(case+'.json')).read_text())==config
        else:
            exclusive(ROOT/(case+'.json'),config)
        files.append(ROOT/(case+'.json'))
    files += [ROOT/n for n in ['source.env','source_runner.sh','run_case.py','runtime_maps.py',
                              'prepare.py','validate_case.py','startup_gate.py','startup_gate/validation.json',
                              'score_cases.py','score_common.py']]
    for repo in ['cbs','glim_ros1','Kimera-VIO','Kimera-VIO-ROS']:
        for pattern in ['*.cpp','*.h','*.hpp','*.launch']:
            files+=sorted(p for p in (ISOLATED/'src'/repo).rglob(pattern) if p.is_file())
    exclusive(ROOT/'protected_manifest.json',{str(p):sha(p) for p in files})
    exclusive(ROOT/'preflight.json',dict(passed=True,integration_tests=8,sanitized_tests=8,
        scope='test TU/header ASan+UBSan; cbs/GTSAM and native estimators not sanitizer-instrumented',
        receiver='unchanged Gaussian no-central-IMU WNOA',root=str(ISOLATED),startup=startup))
if __name__=='__main__': main()
