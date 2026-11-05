"""Complete the retained failed test link with its explicit libdl dependency."""
from pathlib import Path
import json
import os
import subprocess
ROOT=Path('/workspace/cbs_gtsam4.3/runs/local_revision_covariance_cpi1_20261019')
WS=ROOT.parent.parent
old=json.loads((ROOT/'san_commands.json').read_text())
assert old[-1]['name']=='kimera_link' and old[-1]['returncode']==1
assert 'dladdr' in (ROOT/'san_kimera_link.log').read_text()
assert '[  PASSED  ] 50 tests.' in (ROOT/'san_revision_test.log').read_text()
env=os.environ.copy()
env['LD_LIBRARY_PATH']=':'.join(map(str,[
    WS/'runs/shared_linearization_s4_provider_20261012/asan_nopie',
    WS/'runs/shared_linearization_s4_provider_20261012/release_2',
    WS/'runs/e1b4a_local_update_batching_20261012/release_2',
    WS/'runs/e1b_local_profile_20261012/release_2',WS/'devel/lib',
    WS/'devel','/opt/ros/noetic/lib','/usr/local/cuda/lib64']))
env['ASAN_OPTIONS']='detect_leaks=0:halt_on_error=1'
env['UBSAN_OPTIONS']='halt_on_error=1:print_stacktrace=1'
records=[]
for name,command in [('link_libdl',old[-1]['command']+['-ldl']),
                     ('test',[str(ROOT/'san_test_kimera')])]:
    with (ROOT/('san_kimera_completion_'+name+'.log')).open('x') as f:
        result=subprocess.run(command,env=env,cwd=ROOT,stdout=f,stderr=subprocess.STDOUT)
    records.append(dict(name=name,command=command,returncode=result.returncode))
    assert result.returncode==0,name
with (ROOT/'san_completion_passed.json').open('x') as f:
    json.dump(dict(passed=True,commands=records,
        limits='focused instrumentation; GTSAM and CBS libraries uninstrumented; leaks disabled'),f,indent=2)
print('PASS focused sanitizer suite: 50 GLIM/provider tests plus Kimera retirement fixture')
