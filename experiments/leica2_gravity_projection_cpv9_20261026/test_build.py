import hashlib,json,os,subprocess,time,sys
from pathlib import Path
R=Path('/workspace/cbs_gtsam4.3/runs/leica2_gravity_projection_cpv9_20261026')
K=Path('/workspace/cbs_gtsam4.3/runs/kimera_incremental_prior_cpk9_20261019')
W=Path('/workspace/cbs_gtsam4.3')
attempt=sys.argv[1] if len(sys.argv)>1 else '1'
commands=json.loads((K/'actual_test_commands.json').read_text())
compile=[x.replace(str(K/'test.cpp'),str(R/'test.cpp')).replace(str(K/'test.o'),str(R/'test.o')) for x in commands['compile']]
compile.insert(1,'-I'+str(R))
old=json.loads((R/'candidate_commands.json').read_text())[1]
start=old.index('/workspace/cbs_gtsam4.3/devel/lib/libaria_common.so')
link=['/usr/bin/c++',str(R/'test.o'),'-o',str(R/'test'),'-Wl,-rpath,'+str(R)+':'+str(W/'devel/lib'),str(R/'libcbs.so'),*old[start:],str(W/'devel/lib/libgtsam_unstable.so.4.3a1')]
with (R/('test_commands_'+attempt+'.json')).open('x') as f:json.dump([compile,link],f,indent=2)
with (R/('test_build_'+attempt+'.log')).open('x') as f:
 for cmd in (compile,link):subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT,check=True)
assert json.loads((R/'candidate_attempt3_result.json').read_text())['passed']
def run(name,library,args):
 env=dict(os.environ,LD_LIBRARY_PATH=str(library)+':'+str(W/'devel/lib'))
 with (R/(name+'_'+attempt+'.log')).open('x') as f:
  subprocess.run([str(R/'test'),*args],env=env,stdout=f,stderr=subprocess.STDOUT,check=True)
run('test_candidate',R,[])
run('default_candidate',R,['default'])
run('default_original',W/'runs/glim_local_gpu_batching_e1_20261011/private',['default'])
assert (R/('default_original_'+attempt+'.log')).read_bytes()==(R/('default_candidate_'+attempt+'.log')).read_bytes()
result=dict(passed=True,geometry_trials=1000,default_native_and_covariance_byte_identical=True,
 candidate_sha256=hashlib.sha256((R/'libcbs.so').read_bytes()).hexdigest(),
 test_log=(R/('test_candidate_'+attempt+'.log')).read_text(),scope='synthetic actual factor/BPSAM/CP-K9 fixed-lag, not a source capture')
with (R/('test_final_result.json' if attempt=='3' else 'test_result.json')).open('x') as f:json.dump(result,f,indent=2)
print(json.dumps(result))
