"""Private dependency-closure rebuild, frozen instrumentation/math unchanged."""
from pathlib import Path
import argparse
import hashlib
import json
import shlex
import subprocess
import time

WS=Path('/workspace/cbs_gtsam4.3')
ROOT=WS/'runs/e1b_startup_fix_20261012'
OLD=WS/'runs/e1b_local_profile_20261012/release_2'
def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
    p=argparse.ArgumentParser();p.add_argument('--probe-only',action='store_true');p.add_argument('--asan-negative',action='store_true');p.add_argument('--asan-fixed',action='store_true');a=p.parse_args()
    out=ROOT/('layout' if a.probe_only else 'asan_negative' if a.asan_negative else 'asan_fixed' if a.asan_fixed else 'fixed_release');out.mkdir(exist_ok=False)
    cwd=WS/'build/glim_ros';cm=cwd/'CMakeFiles/glim_ros.dir'
    lines=(cm/'flags.make').read_text().splitlines()
    flags=sum([shlex.split(next(s.split(' = ',1)[1] for s in lines if s.startswith(k+' = '))) for k in ['CXX_DEFINES','CXX_INCLUDES','CXX_FLAGS']],[])
    extra=['-I'+str(WS/'runs/e1b_local_profile_20261012'),'-I'+str(WS/'runs/e1b_local_profile_20261012/overlay/include'),'-I'+str(WS/'src/cbs/include'),'-I'+str(WS/'src/gtsam_points/include')]
    commands=[];protected={}
    def run(cmd,name):
        for arg in cmd:
            q=Path(arg) if arg.startswith('/') else cwd/arg
            if q.is_file() and (arg.endswith('.o') or '.so' in q.name) and out not in q.parents: protected[str(q)]=sha(q)
        begin=time.monotonic()
        with (out/(name+'.log')).open('x') as f:r=subprocess.run(cmd,cwd=cwd,stdout=f,stderr=subprocess.STDOUT)
        commands.append(dict(command=cmd,cwd=str(cwd),returncode=r.returncode,seconds=time.monotonic()-begin))
        (out/'commands.json').write_text(json.dumps(commands,indent=2))
        assert r.returncode==0,(name,(out/(name+'.log')).read_text()[-3000:])
        print(name,round(commands[-1]['seconds'],2),flush=True)
    if a.probe_only:
        run(['/usr/bin/c++',*extra,*flags,str(ROOT/'layout_probe.cpp'),'-o',str(out/'layout_probe')],'compile')
        run([str(out/'layout_probe')],'size')
        return
    # Guard all transitive users of the changed layout, not just the edited .cpp.
    users=[]
    for dep in cm.rglob('*.o.d'):
        if '/glim_ros/cbs_bridge.hpp' in dep.read_text(): users.append(str(dep.relative_to(cm))[:-2])
    assert set(users)=={'src/glim_ros/'+n+'.cpp.o' for n in ['glim_ros','cbs_bridge','glim_factor_graph_inspector']},users
    replacements={}
    for name in ([] if a.asan_negative or a.asan_fixed else ['glim_ros','glim_factor_graph_inspector']):
        src=WS/'src/glim_ros1/src/glim_ros'/(name+'.cpp')
        obj=out/(name+'.o')
        run(['/usr/bin/c++',*extra,*flags,'-c',str(src),'-o',str(obj)],name)
        replacements['CMakeFiles/glim_ros.dir/src/glim_ros/'+name+'.cpp.o']=str(obj)
    # Keep the exact already-tested instrumented bridge object, all other DSOs.
    replacements['CMakeFiles/glim_ros.dir/src/glim_ros/cbs_bridge.cpp.o']=str(OLD/'cbs_bridge.cpp.o')
    if a.asan_negative:
        obj=out/'cbs_bridge.asan.o'
        run(['/usr/bin/c++',*extra,*flags,'-O1','-g1','-fsanitize=address','-fno-omit-frame-pointer',
             '-c',str(WS/'runs/e1b_local_profile_20261012/overlay/cbs_bridge.cpp'),'-o',str(obj)],'asan_bridge')
        replacements['CMakeFiles/glim_ros.dir/src/glim_ros/cbs_bridge.cpp.o']=str(obj)
    if a.asan_fixed:
        for name in ['glim_ros','glim_factor_graph_inspector']:
            replacements['CMakeFiles/glim_ros.dir/src/glim_ros/'+name+'.cpp.o']=str(ROOT/'fixed_release'/(name+'.o'))
        replacements['CMakeFiles/glim_ros.dir/src/glim_ros/cbs_bridge.cpp.o']=str(ROOT/'asan_negative/cbs_bridge.asan.o')
    link=shlex.split((cm/'link.txt').read_text());link=[replacements.get(x,x) for x in link]
    link[link.index('-o')+1]=str(out/'libglim_ros.so')
    link+=['-L'+str(OLD),'-le1b_profile','-Wl,-rpath,'+str(OLD)]
    if a.asan_negative or a.asan_fixed: link+=['-fsanitize=address']
    run(link,'link')
    for lib in OLD.glob('lib*.so*'):
        if lib.name!='libglim_ros.so': (out/lib.name).symlink_to(lib)
    assert all(sha(p)==h for p,h in protected.items())
    result=dict(passed=True,header_users=users,protected=protected,libglim_ros_sha256=sha(out/'libglim_ros.so'),
                reused_instrumented_bridge_sha256=sha(OLD/'cbs_bridge.cpp.o'))
    (out/'complete.json').write_text(json.dumps(result,indent=2))
if __name__=='__main__':main()
