#!/usr/bin/env python3
import ast
import hashlib
import json
import shutil
import subprocess
from pathlib import Path
HERE=Path(__file__).resolve().parent
WS=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
ROOT=WS/'runs/glim_e1_real_graph_20261011/prepared'
BASE=WS/'runs/gnssfgo_eth1_vw_jacobian_live160_20261011/prepared'
INSIDE='/workspace/cbs_gtsam4.3/runs/glim_e1_real_graph_20261011/prepared'
def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        for c in iter(lambda:f.read(1048576),b''):h.update(c)
    return h.hexdigest()
assert not (ROOT/'preflight.json').exists()
assert shutil.disk_usage(ROOT).free>10*1024**3
for p in ROOT.glob('*.py'):ast.parse(p.read_text())
for p in ROOT.glob('*.sh'):subprocess.run(['bash','-n',str(p)],check=True)
old=json.loads((HERE.parent/'e1_sync_compatible_batching_20261011/validation_manifest.json').read_text())
protected=json.loads((BASE/'protected_manifest.json').read_text())
changed={p:{'baseline':v,'candidate':sha(p)} for p,v in protected.items() if sha(p)!=v}
assert changed==old['intentional_source_changes']
for p,h in old['prior_candidate_and_baseline_binary_checks'].items():assert sha(p)==h,p

def docker(command,name):
    result=subprocess.run(['docker','exec','cbsms_ws_gpu','bash','-lc',command],capture_output=True,text=True)
    (ROOT/name).write_text(result.stdout+result.stderr)
    assert result.returncode==0,(name,result.returncode,result.stderr[-1000:])
    return result.stdout
docker('nvidia-smi --query-gpu=name,driver_version,memory.used --format=csv','gpu_preflight.log')
docker("python3 -c 'import socket; s=socket.socket(); assert s.connect_ex((\"127.0.0.1\",11401))!=0'",'port_preflight.log')
docker('source /opt/ros/noetic/setup.bash && export LD_LIBRARY_PATH='+INSIDE+'/private_build:/workspace/cbs_gtsam4.3/devel/lib:$LD_LIBRARY_PATH && ldd -r '+INSIDE+'/private_build/libglim_ros.so','ldd_preflight.log')
ldd=(ROOT/'ldd_preflight.log').read_text()
assert 'not found' not in ldd and 'undefined symbol' not in ldd
assert INSIDE+'/private_build/libgtsam_points_cuda.so.1' in ldd
docker('export LD_LIBRARY_PATH='+INSIDE+'/private_build:/workspace/cbs_gtsam4.3/devel/lib:$LD_LIBRARY_PATH && '
       '/workspace/cbs_gtsam4.3/runs/glim_sync_compatible_batching_e1_20261011/private/test_attempt3 '
       '--gtest_output=xml:'+INSIDE+'/regression.xml','regression.log')
assert '[  PASSED  ] 14 tests.' in (ROOT/'regression.log').read_text()
bags={}
for suffix in ['hesai','cpt7_imu','alphasense']:
    p=WS/'src/datasets/GrandTour'/('2024-10-01-11-29-55_'+suffix+'.bag')
    assert p.is_file(); s=p.stat()
    bags[str(p)]={'resolved_path':str(p.resolve()),'bytes':s.st_size,'mtime_ns':s.st_mtime_ns}
    docker('source /opt/ros/noetic/setup.bash && rosbag info --yaml /workspace/cbs_gtsam4.3/src/datasets/GrandTour/'+p.name,
           'bag_'+suffix+'_metadata.log')
pins={str(p):sha(p) for p in ROOT.rglob('*') if p.is_file() and p.suffix in ['.py','.sh','.env','.launch','.h','.cpp','.so']}
pins[str(ROOT/'private_build/libgtsam_points_cuda.so.1.2.1')]=sha(ROOT/'private_build/libgtsam_points_cuda.so.1.2.1')
result={'passed':True,'source_only':True,'duration_s':20,'rate':.1,'port':11401,
        'protected_total':len(protected),'protected_unchanged':len(protected)-len(changed),
        'unchanged_since_prior_e1':True,'prepared_hashes':pins,'bags_metadata_not_full_file_hashes':bags}
with (ROOT/'preflight.json').open('x') as f:json.dump(result,f,indent=2)
print('E1_REAL_PREFLIGHT_PASS: 14 tests, private symbols, GPU, port, frozen inputs')
