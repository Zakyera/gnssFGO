"""No sensor/ROS launch: loader checks, existing tests, original CPU comparison."""
from pathlib import Path
import hashlib
import json
import os
import subprocess
import sys

def main():
    build=Path(sys.argv[1]);out=build/'nonregression';out.mkdir(exist_ok=False)
    ws=Path('/workspace/cbs_gtsam4.3')
    base=[ws/'runs/e1b4a_local_update_batching_20261012/release_2',
          ws/'runs/e1b_local_profile_20261012/release_2',ws/'devel/lib',
          ws/'build/kimera_vio',Path('/opt/ros/noetic/lib'),Path('/usr/local/cuda/lib64')]
    env=os.environ.copy();env.pop('GLIM_SHARED_NATIVE_SHADOW',None)
    env.update(OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1')
    def run(cmd,label,private=True):
        local=env.copy();local['LD_LIBRARY_PATH']=':'.join(map(str,([build] if private else [])+base))
        r=subprocess.run(list(map(str,cmd)),env=local,stdout=subprocess.PIPE,stderr=subprocess.PIPE,timeout=120)
        (out/(label+'.stdout')).write_bytes(r.stdout);(out/(label+'.stderr')).write_bytes(r.stderr)
        assert r.returncode==0,(label,r.returncode,r.stderr[-4000:]);return r.stdout.decode()
    original=run([build/'control'],'original',False)
    candidate=run([build/'control'],'candidate')
    assert original==candidate,'disabled private hooks changed MAIN output'
    ldd=run(['ldd','-r',build/'libglim_ros.so'],'bridge_ldd')
    assert 'undefined symbol' not in ldd and 'not found' not in ldd
    assert str(build/'libgtsam_points.so.1') in ldd
    old_ldd=run(['ldd',build/'control'],'original_ldd',False)
    cpu=next(line.split('=>')[1].strip().split()[0] for line in old_ldd.splitlines() if 'libgtsam_points.so.1 =>' in line)
    assert hashlib.sha256(Path(cpu).read_bytes()).hexdigest()=='a1fac7bede8bf7dafef2f3ae3d7206fd539b0285479d8a6cb739f8da22b0f593'
    regression=run([ws/'runs/e1_correctness_audit_20261011/release_test',
                    '--gtest_output=xml:'+str(out/'regression.xml')],'regression')
    assert '[  PASSED  ] 18 tests.' in regression
    result=dict(disabled_main_byte_identical=True,control_sha256=hashlib.sha256(original.encode()).hexdigest(),
       control_bytes=len(original),original_cpu=cpu,bridge_no_undefined_symbols=True,existing_tests=18,
       private_cpu_sha256=hashlib.sha256((build/'libgtsam_points.so.1.2.1').read_bytes()).hexdigest(),
       private_bridge_sha256=hashlib.sha256((build/'libglim_ros.so').read_bytes()).hexdigest())
    (out/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))
if __name__=='__main__':main()
