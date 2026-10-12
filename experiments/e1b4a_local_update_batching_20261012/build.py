"""Scoped private build. No catkin artifacts, installed DSOs or old outputs written."""
from pathlib import Path
import argparse
import hashlib
import json
import shlex
import shutil
import subprocess
import time

WS=Path('/workspace/cbs_gtsam4.3')
ROOT=WS/'runs/e1b4a_local_update_batching_20261012'
OLD=WS/'runs/e1b_local_profile_20261012'
FIXED=WS/'runs/e1b_startup_fix_20261012/fixed_release'
E1=WS/'runs/glim_sync_compatible_batching_e1_20261011/private'
def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def flags(p):
    lines=p.read_text().splitlines()
    return sum([shlex.split(next(s.split(' = ',1)[1] for s in lines if s.startswith(k+' = ')))
                for k in ['CXX_DEFINES','CXX_INCLUDES','CXX_FLAGS']],[])
def main():
    p=argparse.ArgumentParser();p.add_argument('stage',choices=['libraries','tests'])
    p.add_argument('--variant',choices=['release','ubsan','asan'],default='release')
    p.add_argument('--attempt',default='1');p.add_argument('--test-attempt',default='1')
    p.add_argument('--dispatch-only',action='store_true');a=p.parse_args()
    out=ROOT/(a.variant+'_'+a.attempt)
    if a.stage=='libraries': out.mkdir(exist_ok=False)
    else: assert out.is_dir()
    tag=a.stage+('_'+a.test_attempt if a.stage=='tests' else '')
    snapshot=out/(tag+'_source_snapshot');snapshot.mkdir(exist_ok=False)
    shutil.copytree(ROOT/'authoring',snapshot/'authoring')
    shutil.copytree(ROOT/'overlay',snapshot/'overlay')
    sanitizer=[] if a.variant=='release' else ['-O1','-g1','-fno-omit-frame-pointer',
       '-fsanitize='+('undefined' if a.variant=='ubsan' else 'address,undefined'),'-fno-sanitize-recover=all']
    # Match the retained E1 sanitizer build and the prebuilt dependencies'
    # Eigen allocation ABI. ASan otherwise changes Eigen's auto-detected value.
    if a.variant=='asan':sanitizer+=['-DEIGEN_MALLOC_ALREADY_ALIGNED=1']
    extra=['-I'+str(ROOT/'authoring'),'-I'+str(OLD),'-I'+str(OLD/'overlay/include'),
           '-I'+str(WS/'src/cbs/include'),'-I'+str(WS/'src/gtsam_points/include')]
    commands=[];protected={}
    def run(cmd,cwd,name):
        for arg in cmd:
            q=Path(arg) if arg.startswith('/') else cwd/arg
            if q.is_file() and (arg.endswith('.o') or '.so' in q.name) and out not in q.parents:
                protected[str(q)]=sha(q)
        begin=time.monotonic()
        with (out/(tag+'_'+name+'.log')).open('x') as f:
            result=subprocess.run(cmd,cwd=cwd,stdout=f,stderr=subprocess.STDOUT)
        commands.append(dict(command=cmd,cwd=str(cwd),returncode=result.returncode,seconds=time.monotonic()-begin))
        (out/(tag+'_commands.json')).write_text(json.dumps(commands,indent=2))
        (out/(tag+'_protected.json')).write_text(json.dumps(protected,indent=2))
        print(name,result.returncode,round(commands[-1]['seconds'],2),flush=True)
        if result.returncode: print((out/(tag+'_'+name+'.log')).read_text()[-6000:]);raise SystemExit(result.returncode)
    linkextra=['-L'+str(out),'-L'+str(OLD/'release_2'),'-le1b_profile',
               '-Wl,-rpath,'+str(out)+':'+str(OLD/'release_2')]+sanitizer
    if a.stage=='libraries':
        records=json.loads((OLD/'release_2/commands.json').read_text())
        cwd=WS/'build/gtsam/gtsam';gflags=flags(cwd/'CMakeFiles/gtsam.dir/flags.make')
        for src in ['NonlinearFactorGraph.cpp','ScopedGraphLinearizer.cpp']:
            path=ROOT/('overlay' if src.startswith('Nonlinear') else 'authoring')/src
            run(['/usr/bin/c++',*extra,*gflags,*sanitizer,'-c',str(path),'-o',str(out/(src+'.o'))],cwd,src)
        record=next(r for r in records if r['label']=='gtsam_link')
        cmd=[str(out/'NonlinearFactorGraph.cpp.o') if x==str(OLD/'release_2/NonlinearFactorGraph.cpp.o') else x for x in record['command']]
        dest=out/'libgtsam.so.4.3a1';cmd[cmd.index('-o')+1]=str(dest)
        cmd+=[str(out/'ScopedGraphLinearizer.cpp.o')]+linkextra
        run(cmd,cwd,'gtsam_link')
        for name in ['libgtsam.so','libgtsam.so.4']:(out/name).symlink_to(dest.name)
        if a.variant=='release':
            cwd=WS/'build/glim_ros';cm=cwd/'CMakeFiles/glim_ros.dir';gflags=flags(cm/'flags.make')
            users=[str(dep.relative_to(cm))[:-2] for dep in cm.rglob('*.o.d')
                   if '/glim_ros/cbs_bridge.hpp' in dep.read_text()]
            assert set(users)=={'src/glim_ros/'+s+'.cpp.o' for s in ['glim_ros','cbs_bridge','glim_factor_graph_inspector']},users
            replacements={}
            for name in ['glim_ros','glim_factor_graph_inspector','cbs_bridge']:
                src=ROOT/'overlay/cbs_bridge.cpp' if name=='cbs_bridge' else WS/'src/glim_ros1/src/glim_ros'/(name+'.cpp')
                obj=out/(name+'.cpp.o')
                run(['/usr/bin/c++',*extra,*gflags,'-c',str(src),'-o',str(obj)],cwd,name)
                replacements['CMakeFiles/glim_ros.dir/src/glim_ros/'+name+'.cpp.o']=str(obj)
            cmd=[replacements.get(x,x) for x in shlex.split((cm/'link.txt').read_text())]
            cmd[cmd.index('-o')+1]=str(out/'libglim_ros.so');cmd+=linkextra
            run(cmd,cwd,'glim_ros_link')
            (out/'header_users.json').write_text(json.dumps(users,indent=2))
        for lib in FIXED.glob('lib*.so*'):
            if lib.name not in {'libgtsam.so','libgtsam.so.4','libgtsam.so.4.3a1','libglim_ros.so'}:
                (out/lib.name).symlink_to(lib.resolve())
    else:
        base=json.loads((E1/'test_attempt3_commands.json').read_text())[0]
        for source in (['test_dispatch.cpp'] if a.dispatch_only else ['test_dispatch.cpp','test_solver.cpp']):
            path=ROOT/'authoring'/source
            cmd=[str(path) if x.endswith('/cbs_local_graph_linearizer_test.cpp') else x for x in base]
            cmd[cmd.index('-o')+1]=str(out/(Path(source).stem+'_'+a.test_attempt))
            cmd[1:1]=extra+sanitizer+['-L'+str(out)]
            cmd+=linkextra+['-ltbb']
            run(cmd,ROOT,source)
    assert all(sha(p)==h for p,h in protected.items())
    (out/(tag+'_complete.json')).write_text(json.dumps(dict(protected_unchanged=len(protected),
        artifacts={p.name:sha(p) for p in out.iterdir() if p.is_file() and not p.is_symlink() and p.suffix not in ['.log','.json']}),indent=2)+'\n')
if __name__=='__main__':main()
