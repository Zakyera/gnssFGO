"""Build standalone saved graph reader against pinned historical libraries."""
import json,subprocess,hashlib
from pathlib import Path
B=Path(__file__).resolve().parent;H=B/'saved_native_attempt2';H.mkdir(exist_ok=False)
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3');K=W/'runs'/B.name;A=K/'saved_native_attempt2';A.mkdir(exist_ok=False)
C=Path('/workspace/cbs_gtsam4.3');CK=C/'runs'/B.name;CA=CK/'saved_native_attempt2';CP=C/'runs/eig2_hesai_insertion_cpv51_20261030/prepared'
def save(n,v):
 with (H/n).open('x') as f:json.dump(v,f,indent=2)
def call(n,cmd):
 save(n+'_command.json',cmd)
 with (H/(n+'.log')).open('x') as f:r=subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT)
 save(n+'_receipt.json',dict(exit=r.returncode));assert r.returncode==0,(n,r.returncode)
states=json.loads(subprocess.check_output(['docker','inspect','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia']));save('preflight.json',[dict(name=x['Name'],running=x['State']['Running']) for x in states]);assert all(not x['State']['Running'] for x in states)
original=B.parent/'eig2_first_rotation_cpv53_20261030'
cc=json.loads((original/'release_compile_command.json').read_text());cc=[str(CK/'SavedFrame_v2.cpp') if x.endswith('/Extract.cpp') else str(CA/'release.o') if x.endswith('/release.o') else x for x in cc];cc.insert(1,'-I'+str(C/'src/Kimera-VIO/include'))
link=json.loads((original/'release_link_command.json').read_text());link=[str(CA/'release.o') if x.endswith('/release.o') else str(CA/'release') if x.endswith('/release') else x for x in link]
link.extend(['-Wl,-rpath,'+str(CP/'private_build'),str(CP/'private_build/libkimera_vio.so'),'-lglog','-lgflags'])
link=[x for x in link if 'opencv' not in x]+['/usr/lib/x86_64-linux-gnu/libopencv_core.so.4.2']
call('start',['docker','start','cbsms_ws_gpu'])
try:
 call('compile',['docker','exec','cbsms_ws_gpu',*cc]);call('link',['docker','exec','cbsms_ws_gpu',*link])
 for mode in ['off','on']:
  call('extract_'+mode,['docker','exec','-e','LD_LIBRARY_PATH='+str(CP/'private_build')+':'+str(C/'devel/lib'),'cbsms_ws_gpu',str(CA/'release'),str(CP/f'full_{mode}_source/checkpoints/update_6/before_ordinary/checkpoint.bin'),str(CP/'input_profiles/kimera_profile'),str(CA/mode)])
  assert (A/mode/'before.bin').read_bytes()==(A/mode/'after.bin').read_bytes()
 save('results.json',dict(passed=True,zero_optimizer_calls=True,byte_nonmutation=True,modes={m:json.loads((A/m/'result.json').read_text()) for m in ['off','on']}))
finally:call('restore',['docker','stop','cbsms_ws_gpu'])
