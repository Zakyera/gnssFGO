"""Exclusive private build; all inherited sources/libraries are read-only."""
import argparse, hashlib, json, os, signal, subprocess, time, shutil
from pathlib import Path
W=Path('/workspace/cbs_gtsam4.3')
R=Path(__file__).resolve().parent
I=W/'runs/local_revision_covariance_cpi1_20261019'
S=W/'runs/shared_linearization_s4_provider_20261012/release_2'
K=W/'runs/main_gpu_batching_cpk16_20261020'
P=W/'runs/native_covariance_cache_cpi3_20261019/release_v2'
B=W/'runs/e1b4a_local_update_batching_20261012/release_2'
O=W/'runs/e1b_local_profile_20261012'

def sha(p):
    with p.open('rb') as f:return hashlib.sha256(f.read()).hexdigest()

def environment(mode,out):
    env=os.environ.copy()
    for key in list(env):
        if key.startswith(('GLIM_SHARED_NATIVE','GLIM_REVISION_COVARIANCE','CPK16_','CPV82_','CBS_E1B_')):env.pop(key)
    lib=K/('ubsan_1' if mode=='ubsan' else 'release_1')
    env.update(LD_LIBRARY_PATH=':'.join(map(str,[out,lib,I,S,P,B,O/'release_2',W/'devel/lib',W/'devel',Path('/opt/ros/noetic/lib'),Path('/usr/local/cuda/lib64')])),
        OMP_NUM_THREADS='1',UBSAN_OPTIONS='halt_on_error=1:print_stacktrace=1',
        ROS_MASTER_URI='http://localhost:11582',ROS_HOSTNAME='localhost',
        ROS_LOG_DIR=str(out/'ros_logs'))
    return env

def main():
    parser=argparse.ArgumentParser();parser.add_argument('mode',choices=['release','ubsan']);parser.add_argument('attempt');a=parser.parse_args()
    out=R/(a.mode+'_'+a.attempt);out.mkdir(exist_ok=False)
    (out/'inputs').mkdir()
    for path in R.iterdir():
        if path.is_file():shutil.copy2(path,out/'inputs'/path.name)
    env=environment(a.mode,out);records=[]
    def run(name,cmd,cwd=out):
        started=time.monotonic()
        with (out/(name+'.log')).open('x') as log:
            code=subprocess.call(cmd,cwd=cwd,env=env,stdout=log,stderr=subprocess.STDOUT)
        records.append(dict(name=name,command=cmd,cwd=str(cwd),returncode=code,seconds=time.monotonic()-started))
        (out/'commands.json').write_text(json.dumps(records,indent=2))
        print(name,code,flush=True)
        if code:
            print((out/(name+'.log')).read_text()[-6000:],flush=True)
            raise SystemExit(code)
    def flags(cmd):
        if a.mode=='ubsan':cmd += ['-O1','-g1','-fsanitize=undefined','-fno-sanitize-recover=all','-fno-omit-frame-pointer']
        return cmd
    old={r['name']:r for r in json.loads((I/'release_commands.json').read_text())}
    cc=old['bridge_compile']['command'].copy()
    cc=[x.replace(str(I/'cbs_bridge.cpp'),str(R/'cbs_bridge.cpp')) if x==str(I/'cbs_bridge.cpp') else x for x in cc]
    cc[cc.index('-o')+1]=str(out/'bridge.o');cc.insert(1,'-I'+str(R))
    run('bridge_compile',flags(cc),old['bridge_compile']['cwd'])
    link=old['bridge_link']['command'].copy()
    link=[str(out/'bridge.o') if x==str(I/'cbs_bridge.cpp.o') else x for x in link]
    link[link.index('-o')+1]=str(out/'libglim_ros.so')
    run('bridge_link',flags(link),old['bridge_link']['cwd'])
    tc=old['bridge_compile']['command'].copy()
    tc=[str(R/'test_bridge.cpp') if x==str(I/'cbs_bridge.cpp') else x for x in tc]
    tc[tc.index('-o')+1]=str(out/'test_bridge.o');tc[1:1]=['-I'+str(R),'-fno-access-control']
    run('bridge_test_compile',flags(tc),old['bridge_compile']['cwd'])
    tl=[x for x in old['bridge_link']['command'] if not x.endswith('.o') and x not in ['-shared','-Wl,-soname,libglim_ros.so']]
    tl[tl.index('-o')+1]=str(out/'test_bridge')
    tl[1:1]=[str(out/'test_bridge.o'),'-L'+str(out),'-lglim_ros','-lgtest_main','-lgtest']
    # The estimator DSO recipe intentionally permits runtime-selected overlays;
    # an executable must resolve those same pinned symbols at link time too.
    overlay={str(W/'devel/lib/libgtsam_points.so.1.2.1'):str(K/('ubsan_1' if a.mode=='ubsan' else 'release_1')/'libgtsam_points.so.1.2.1'),
             str(W/'devel/lib/libcbs.so'):str(B/'libcbs.so'),
             str(W/'devel/lib/libgtsam.so.4.3a1'):str(B/'libgtsam.so.4.3a1'),
             str(W/'devel/lib/libgtsam_points_cuda.so.1.2.1'):str(O/'release_2/libgtsam_points_cuda.so.1.2.1')}
    tl=[overlay.get(x,x) for x in tl]
    run('bridge_test_link',flags(tl),old['bridge_link']['cwd'])
    run('bridge_test_ldd',['ldd',str(out/'test_bridge')])
    master_log=(out/'roscore.log').open('x')
    master=subprocess.Popen(['/opt/ros/noetic/bin/roscore','-p','11582'],env=env,stdout=master_log,stderr=subprocess.STDOUT,start_new_session=True)
    (out/'master.json').write_text(json.dumps(dict(pid=master.pid,started=time.time(),command=['roscore','-p','11582'])))
    try:
        import xmlrpc.client
        deadline=time.monotonic()+25
        while True:
            assert master.poll() is None,'ROS master exited'
            try:
                assert xmlrpc.client.ServerProxy(env['ROS_MASTER_URI']).getPid('/cpv82')[0]==1
                break
            except Exception:
                if time.monotonic()>deadline:raise
                time.sleep(.2)
        run('bridge_tests',[str(out/'test_bridge'),'--gtest_output=xml:'+str(out/'tests.xml')])
        original_ld=env['LD_LIBRARY_PATH']
        for label,opt,original in [('original','0',True),('candidate_disabled','0',False),('candidate_enabled','1',False)]:
            env['CPV82_PARITY_PATH']=str(out/(label+'.txt'))
            env['CPV82_GLIM_PERSISTENT_COMPAT']=opt
            env['LD_LIBRARY_PATH']=(str(I)+':'+original_ld) if original else original_ld
            run('parity_'+label,[str(out/'test_bridge'),'--gtest_filter=PersistentCompatibility.TemporaryDisabledParity'])
        env['LD_LIBRARY_PATH']=original_ld
        env.pop('CPV82_PARITY_PATH');env.pop('CPV82_GLIM_PERSISTENT_COMPAT')
        assert (out/'original.txt').read_bytes()==(out/'candidate_disabled.txt').read_bytes()==(out/'candidate_enabled.txt').read_bytes()
        (out/'parity.json').write_text(json.dumps(dict(passed=True,exact=True,sha256=sha(out/'original.txt'))))
    finally:
        if master.poll() is None:
            os.killpg(master.pid,signal.SIGINT)
            try:master.wait(timeout=20)
            except subprocess.TimeoutExpired:os.killpg(master.pid,signal.SIGTERM);master.wait(timeout=10)
        master_log.close()
        (out/'master_cleanup.json').write_text(json.dumps(dict(exited=True,returncode=master.returncode)))
    # Derived real GPU/IMU wrapper test, now with persistent peers and 5s lag.
    gpu_recipe=json.loads((K/'k16b_build_5/commands.json').read_text())
    recipe=next(row for row in gpu_recipe if row['mode']==('ubsan' if a.mode=='ubsan' else 'candidate'))['command'].copy()
    recipe=[str(R/'test_persistent_gpu.cpp') if x.endswith('/test_main_lifecycle.cpp') else x for x in recipe]
    recipe[recipe.index('-o')+1]=str(out/'test_persistent_gpu')
    run('gpu_compile',recipe)
    geometry=K/'k16b_checks_4/frozen_geometry.bin';assert geometry.exists()
    for enabled in ['0','1']:
        run('gpu_'+enabled,[str(out/'test_persistent_gpu'),str(out/('gpu_'+enabled+'.txt')),enabled,str(geometry)])
    run('gpu_equivalence',['python3',str(K/'k16b_source/compare_k16b.py'),str(out/'gpu_0.txt'),str(out/'gpu_1.txt')])
    (out/'passed.json').write_text(json.dumps(dict(passed=True,mode=a.mode,
        bridge_sha256=sha(out/'libglim_ros.so'),input_geometry_sha256=sha(geometry),
        note='Synthetic compatibility, not sensor accuracy or covariance calibration. Vendor libraries not wholly sanitizer-instrumented.'),indent=2))

if __name__=='__main__':main()
