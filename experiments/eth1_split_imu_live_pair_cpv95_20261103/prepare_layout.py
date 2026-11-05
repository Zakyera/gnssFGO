"""Copy/rebase static qualified harness only. Never starts a container or capture."""
import json, shutil
from pathlib import Path
H=Path(__file__).resolve().parent;E=H.parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3');P=W/'runs'/H.name/'prepared'
old84='eig2_symmetric_persistent_full_cpv84_20261101_attempt1'
old={'off':'eth1_alpha_bias_rw_full_off_cpv89_20261102','on':'eth1_split_imu_persistent_on_cpv91_20261102_attempt1'}
assert not P.exists();P.mkdir(parents=True)
receipts=[]
def copied(src,dst,text_replacements=()):
    dst.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(src,dst)
    if text_replacements:
        text=dst.read_text()
        for a,b in text_replacements:text=text.replace(a,b)
        dst.write_text(text)
    receipts.append(dict(source=str(src),destination=str(dst)))
for mode,name in old.items():
    orig=W/'runs'/name/'prepared';dest=P/mode
    replacements=[(name+'/prepared/'+mode+'_source',H.name+'/prepared/full_'+mode+'_source'),
                  (name+'/prepared',H.name+'/prepared/'+mode),(name,H.name)]
    for f in orig.iterdir():
        if f.is_file() and f.name!='protected_manifest.json':
            copied(f,dest/f.name,replacements if f.suffix in ('.py','.sh','.launch','.env') else ())
    for folder in ('input_profiles','private_build'):
        shutil.copytree(orig/folder,dest/folder,copy_function=shutil.copy2)
    env=dest/'source.env';text=env.read_text()
    text=text.replace('CBSMS_RUN_ROOT=/workspace/cbs_gtsam4.3/runs/'+H.name+'/prepared/'+mode,
                      'CBSMS_RUN_ROOT=/workspace/cbs_gtsam4.3/runs/'+H.name+'/prepared')
    text=text.replace('CBSMS_USE_EXISTING_ROS_MASTER=false','CBSMS_USE_EXISTING_ROS_MASTER=true')
    import re
    text=re.sub(r'CBSMS_ROS_MASTER_PORT=\d+','CBSMS_ROS_MASTER_PORT='+str(11595+(mode=='on')),text)
    env.write_text(text)
    runner=dest/'source_runner_on.sh'
    runner.write_text(runner.read_text().replace('${run_root}/private_build','${experiment_dir}/private_build'))
for f in ('input_audit.json','expected_headers.json'):
    copied(P/'off'/f,P/f)
    assert (P/'off'/f).read_bytes()==(P/'on'/f).read_bytes(),f
orig84=W/'runs'/old84/'prepared'
for f in ('experiment.py','run_case_on.py','receiver_cpk19.sh','validate_case.py','validate_local.py','expiry_audit.py','eof_boundary.py'):
    copied(orig84/f,P/f,[(old84,H.name)])
for f in (orig84/'transport_tools').iterdir():
    if f.is_file():copied(f,P/'transport_tools'/f.name,[(old84,H.name)])
for mode in ('off','on'):
    config=json.loads((orig84/'full_on.json').read_text())
    config['sequence']='ETH1 CP-V95 full346.32s live split-IMU CBS '+mode.upper()
    config['ros_master_uri']='http://127.0.0.1:'+str(11595+(mode=='on'))
    config['receiver_binary']=str(P/'receiver_cpk19.sh')
    for s in config['sources']:
        s['session']='eth1_cpv95_'+mode+'_20261103'
        s['native_body_frame']='alphasense_imu' if s['kind']=='kimera' else 'cpt7_imu'
        s['native_to_wire_conversion']='alpha_to_cpt7_conjugation_full_covariance' if s['kind']=='kimera' else 'identity'
    (P/('full_'+mode+'.json')).write_text(json.dumps(config,indent=2)+'\n')
copied(E/'eth1_split_imu_fusion_cpv94_20261102/frame_adapter.py',P/'transport_tools/frame_adapter.py',
       [("Path('/home/yeranis/repos/gnssFGO/experiments/fixed_lag_eig2_g2k_reference_fl4y_20261017/analyze.py')","Path(__file__).resolve().with_name('frame_geometry.py')")])
copied(E/'fixed_lag_eig2_g2k_reference_fl4y_20261017/analyze.py',P/'transport_tools/frame_geometry.py')
audit=json.loads((P/'input_audit.json').read_text())
(P/'transport_tools/frame_calibration.json').write_text(json.dumps(dict(
    native='alphasense_imu',wire='cpt7_imu',T_alphasense_cpt7=audit['T_alphasense_cpt7'],
    source=str(E/old['on']/'prepared.json'),convention='T_Alpha_CPT7; rotation-translation tangent; native diagnostic rows ONLY'),indent=2)+'\n')
for mode in ('off','on'):
    copied(E/old['on']/'frame_audit.py',H/('frame_'+mode+'.py'),[(old['on'],H.name)])
for f in ('launch.py','post_closeout.py'):
    copied(E/old84/f,H/f,[(old84,H.name),('EIG2-CPV84-A1','ETH1-CPV95')])
with (H/'layout_receipt.json').open('x') as f:json.dump(dict(passed=True,copies=receipts),f,indent=2)
print(P)
