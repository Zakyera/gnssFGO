"""Exercise mixed pose/velocity/bias recording with the existing IMU fixture."""
from pathlib import Path
import json
import os
import subprocess

ROOT=Path('/workspace/cbs_gtsam4.3/runs/shared_linearization_s3_capture_20261012')
BUILD=ROOT/'release_1'
out=ROOT/'mixed_fixture';out.mkdir(exist_ok=False)
s=(BUILD/'source/test_lifecycle.cpp').read_text()
a='  Main main(.25,params());auto o=attach(&main);'
assert s.count(a)==1
s=s.replace(a,a+'\n  o->setRecording("'+str(out/'imu.bin')+'");')
a='      ASSERT_TRUE(result.ok)<<result.message;EXPECT_TRUE(cov.relative.allFinite());'
assert s.count(a)==1
s=s.replace(a,a+'\n      o->recordQueryInput(x(i-1),x(i),main.calculateEstimate<Pose3>(x(i-1)),main.calculateEstimate<Pose3>(x(i)));\n      o->recordQueryOutput(cov,0.);')
src=out/'test_mixed.cpp';src.write_text(s)
commands=json.loads((BUILD/'commands.json').read_text())
compile=next(r for r in commands if str(BUILD/'test_lifecycle.o') in r['command'] and '-c' in r['command'])['command']
compile=[str(src) if x==str(BUILD/'source/test_lifecycle.cpp') else str(out/'test_mixed.o') if x==str(BUILD/'test_lifecycle.o') else x for x in compile]
link=next(r for r in commands if '-o' in r['command'] and str(BUILD/'test_shadow') in r['command'])['command']
link=[str(out/'test_mixed.o') if x==str(BUILD/'test_lifecycle.o') else str(out/'test_mixed') if x==str(BUILD/'test_shadow') else x for x in link]
(out/'commands.json').write_text(json.dumps([compile,link],indent=2))
with (out/'build.log').open('x') as log:
    for cmd in [compile,link]:subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT,check=True)
env=os.environ.copy();env['LD_LIBRARY_PATH']=str(BUILD)+':/workspace/cbs_gtsam4.3/runs/e1b4a_local_update_batching_20261012/release_2:/workspace/cbs_gtsam4.3/runs/e1b_local_profile_20261012/release_2:/workspace/cbs_gtsam4.3/devel/lib:/usr/local/cuda/lib64'
with (out/'test.log').open('x') as log:
    subprocess.run([str(out/'test_mixed'),'--gtest_filter=Lifecycle.ActualNativeImuVelocityBiasAndPoseRetireTogether'],env=env,stdout=log,stderr=subprocess.STDOUT,check=True)
print((out/'test.log').read_text())
