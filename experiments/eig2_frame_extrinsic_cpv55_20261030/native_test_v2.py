"""Fresh build-only attempt: use GTSAM's pinned Eigen headers, no test changes."""
import json,subprocess,hashlib
from pathlib import Path
BASE=Path(__file__).resolve().parent;H=BASE/'native_attempt1';H.mkdir(exist_ok=False)
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3');K=W/'runs'/BASE.name;A=K/'native_attempt1';A.mkdir(exist_ok=False)
C=Path('/workspace/cbs_gtsam4.3');CK=C/'runs'/BASE.name;CA=CK/'native_attempt1'
def save(n,v):
 with (H/n).open('x') as f:json.dump(v,f,indent=2)
def call(n,cmd):
 save(n+'_command.json',cmd)
 with (H/(n+'.log')).open('x') as f:r=subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT)
 save(n+'_receipt.json',dict(exit=r.returncode));assert r.returncode==0,(n,r.returncode)
states=json.loads(subprocess.check_output(['docker','inspect','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia']))
save('container_preflight.json',[dict(name=x['Name'],running=x['State']['Running']) for x in states]);assert all(not x['State']['Running'] for x in states)
call('start_test_container',['docker','start','cbsms_ws_gpu'])
try:
 for mode,flags in [('release',['-O2','-DNDEBUG']),('san',['-O1','-g','-fsanitize=address,undefined','-fno-omit-frame-pointer'])]:
  cmd=['c++','-std=c++17',*flags,'-I'+str(C/'devel/include'),'-I'+str(C/'devel/include/gtsam/3rdparty/Eigen'),'-I/usr/include/eigen3','-I'+str(C/'src/Kimera-VIO-ROS/include'),str(CK/'FrameTest.cpp'),str(C/'src/Kimera-VIO-ROS/src/CbsRelativeFrameConversion.cpp'),'-L'+str(C/'devel/lib'),'-Wl,-rpath,'+str(C/'devel/lib'),'-lgtsam','-o',str(CA/mode)]
  call(mode+'_compile',['docker','exec','cbsms_ws_gpu',*cmd])
  call(mode+'_test',['docker','exec','cbsms_ws_gpu',str(CA/mode)])
 results={m:json.loads((H/(m+'_test.log')).read_text()) for m in ['release','san']}
 save('native_test.json',dict(passed=True,results=results,files={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [K/'FrameTest.cpp',A/'release',A/'san',W/'src/Kimera-VIO-ROS/src/CbsRelativeFrameConversion.cpp',W/'src/Kimera-VIO-ROS/include/kimera_vio_ros/CbsRelativeFrameConversion.h']}))
finally:call('restore_test_container',['docker','stop','cbsms_ws_gpu'])
