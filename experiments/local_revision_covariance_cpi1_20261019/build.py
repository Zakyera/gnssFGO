"""Build ONLY a private bridge and synthetic tests using saved ABI recipes."""
from pathlib import Path
import argparse
import hashlib
import json
import os
import subprocess
import time

ROOT=Path(__file__).resolve().parent
WS=Path('/workspace/cbs_gtsam4.3')
BASE=WS/'runs/shared_linearization_s4_provider_20261012/release_2'
KIM=WS/'runs/fixed_lag_current_chart_fl4q_20261016'
RECIPES=json.loads((BASE/'commands.json').read_text())

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--sanitize',action='store_true')
    parser.add_argument('--attempt',default='');parser.add_argument('--tests-only',action='store_true');a=parser.parse_args()
    tag=('san' if a.sanitize else 'release')+a.attempt
    receipt=ROOT/(tag+'_commands.json'); assert not receipt.exists(), 'refuse overwrite'
    records=[]
    env=os.environ.copy()
    env['LD_LIBRARY_PATH']=':'.join(map(str,[BASE,
        WS/'runs/e1b4a_local_update_batching_20261012/release_2',
        WS/'runs/e1b_local_profile_20261012/release_2',WS/'devel/lib',
        WS/'devel','/opt/ros/noetic/lib','/usr/local/cuda/lib64']))
    if a.sanitize:
        env['LD_LIBRARY_PATH']=str(BASE.parent/'asan_nopie')+':'+env['LD_LIBRARY_PATH']
    env['OMP_NUM_THREADS']='1'
    for key in ['GLIM_SHARED_NATIVE_PROVIDER','GLIM_SHARED_NATIVE_SHADOW','GLIM_REVISION_COVARIANCE_POLICY']:
        env.pop(key,None)
    env['ASAN_OPTIONS']='detect_leaks=0:halt_on_error=1'
    env['UBSAN_OPTIONS']='halt_on_error=1:print_stacktrace=1'
    def run(name,command,cwd=ROOT):
        log=ROOT/(tag+'_'+name+'.log'); assert not log.exists()
        start=time.monotonic()
        with log.open('x') as out:
            code=subprocess.call(command,cwd=cwd,env=env,stdout=out,stderr=subprocess.STDOUT)
        records.append(dict(name=name,command=command,cwd=str(cwd),returncode=code,seconds=time.monotonic()-start))
        receipt.write_text(json.dumps(records,indent=2)+'\n')
        print(name,code,flush=True)
        if code: raise SystemExit('FAILED: '+str(log))
    def options(cmd):
        if a.sanitize:
            cmd=[x.replace('-L'+str(BASE),'-L'+str(BASE.parent/'asan_nopie')) for x in cmd]
            cmd=[x for x in cmd if x not in ['-O2','-O3','-fPIC']]
            cmd += ['-O0','-g1','-fsanitize=address,undefined','-fno-omit-frame-pointer','-fno-pie','-no-pie']
            # Match the unsanitized pinned Eigen allocator ABI.
            cmd += ['-DEIGEN_MALLOC_ALREADY_ALIGNED=1','-ldl']
        return cmd
    if not a.sanitize and not a.tests_only:
        compile_cmd=RECIPES[5]['command'].copy()
        compile_cmd.insert(1,'-I'+str(ROOT))
        compile_cmd=[x.replace(str(BASE/'overlay/cbs_bridge.cpp'),str(ROOT/'cbs_bridge.cpp')).replace(
            str(BASE/'cbs_bridge.cpp.o'),str(ROOT/'cbs_bridge.cpp.o')) for x in compile_cmd]
        run('bridge_compile',compile_cmd,RECIPES[5]['cwd'])
        link=[x.replace(str(BASE/'cbs_bridge.cpp.o'),str(ROOT/'cbs_bridge.cpp.o')).replace(
            str(BASE/'libglim_ros.so'),str(ROOT/'libglim_ros.so')) for x in RECIPES[6]['command']]
        run('bridge_link',link,RECIPES[6]['cwd'])
    cmd=RECIPES[7]['command'].copy();cmd.insert(1,'-I'+str(ROOT))
    cmd=[x.replace(str(BASE/'source/test_lifecycle.cpp'),str(ROOT/'test_revision.cpp')).replace(
        str(BASE/'test_lifecycle.o'),str(ROOT/(tag+'_test_revision.o'))) for x in cmd]
    run('revision_compile',options(cmd))
    cmd=RECIPES[9]['command'].copy()
    cmd=[x.replace(str(BASE/'test_lifecycle.o'),str(ROOT/(tag+'_test_revision.o'))).replace(
        str(BASE/'test_shadow'),str(ROOT/(tag+'_test_revision'))) for x in cmd]
    run('revision_link',options(cmd))
    run('revision_test',[str(ROOT/(tag+'_test_revision'))])
    if not a.sanitize: run('revision_repeat',[str(ROOT/(tag+'_test_revision'))])
    kr=json.loads((KIM/'actual_test_commands.json').read_text())
    cmd=[x.replace(str(KIM/'test.cpp'),str(ROOT/'test_kimera.cpp')).replace(
        str(KIM/'test.o'),str(ROOT/(tag+'_test_kimera.o'))) for x in kr['compile']]
    run('kimera_compile',options(cmd))
    cmd=[x.replace(str(KIM/'test.o'),str(ROOT/(tag+'_test_kimera.o'))).replace(
        str(KIM/'test'),str(ROOT/(tag+'_test_kimera'))) for x in kr['link']]
    run('kimera_link',options(cmd))
    run('kimera_test',[str(ROOT/(tag+'_test_kimera'))])
    (ROOT/(tag+'_passed.json')).write_text(json.dumps(dict(passed=True,
        scope='synthetic tests; sanitizer covers new helper/test code plus existing instrumented native provider, not vendor GTSAM/CBS',
        commands_sha256=hashlib.sha256(receipt.read_bytes()).hexdigest()),indent=2)+'\n')

if __name__=='__main__':main()
