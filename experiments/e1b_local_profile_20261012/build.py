#!/usr/bin/env python3
"""Isolated relink of changed host units; old objects/libraries remain read-only."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shlex
import subprocess
import time

ROOT=Path(__file__).resolve().parent
WS=Path('/workspace/cbs_gtsam4.3')
OLD=WS/'runs/glim_sync_compatible_batching_e1_20261011/private'

def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        for b in iter(lambda:f.read(1048576),b''):h.update(b)
    return h.hexdigest()

def flags(path):
    lines=path.read_text().splitlines()
    return sum([shlex.split(next(x.split(' = ',1)[1] for x in lines if x.startswith(key+' = ')))
                for key in ('CXX_DEFINES','CXX_INCLUDES','CXX_FLAGS')],[])

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--variant',choices=['release','ubsan'],default='release')
    parser.add_argument('--attempt',default='1');args=parser.parse_args()
    out=ROOT/(args.variant+'_'+args.attempt);out.mkdir(exist_ok=False)
    extra=['-I'+str(ROOT),'-I'+str(ROOT/'overlay/include'),
           '-I'+str(WS/'src/cbs/include'),'-I'+str(WS/'src/gtsam_points/include')]
    sanitize=['-fsanitize=undefined','-fno-sanitize-recover=all','-fno-omit-frame-pointer'] if args.variant=='ubsan' else []
    commands=[];protected={}
    def run(cmd,cwd,label):
        for arg in cmd:
            q=Path(arg) if arg.startswith('/') else Path(cwd)/arg
            if (arg.endswith('.o') or '.so' in q.name) and q.is_file() and out not in q.parents:
                protected[str(q)]=sha(q)
        start=time.monotonic()
        with (out/(label+'.log')).open('x') as log:
            code=subprocess.run(cmd,cwd=cwd,stdout=log,stderr=subprocess.STDOUT).returncode
        commands.append(dict(label=label,command=cmd,cwd=str(cwd),seconds=time.monotonic()-start,returncode=code))
        (out/'commands.json').write_text(json.dumps(commands,indent=2)+'\n')
        (out/'protected.json').write_text(json.dumps(protected,indent=2)+'\n')
        print(label,code,round(commands[-1]['seconds'],2),flush=True)
        if code:
            print((out/(label+'.log')).read_text()[-6000:]);raise SystemExit(code)
    common=['/usr/bin/c++','-std=c++17','-O3','-DNDEBUG','-fPIC','-pthread']+sanitize
    run(common+['-shared',str(ROOT/'RuntimeProfile.cpp'),'-Wl,-soname,libe1b_profile.so','-o',str(out/'libe1b_profile.so')],ROOT,'profile')
    profile_link=['-L'+str(out),'-le1b_profile','-Wl,-rpath,'+str(out)]+sanitize
    targets=[
      ('gtsam',WS/'build/gtsam/gtsam','gtsam', [('nonlinear/ISAM2.cpp','ISAM2.cpp'),('nonlinear/NonlinearFactorGraph.cpp','NonlinearFactorGraph.cpp')]),
      ('cbs',WS/'build/cbs','cbs',[('src/bpsam/bpsam.cpp','bpsam.cpp')]),
      ('glim',WS/'build/glim','glim',[('src/glim/odometry/odometry_estimation_imu.cpp','odometry_estimation_imu.cpp')]),
      ('glim_ros',WS/'build/glim_ros','glim_ros',[('src/glim_ros/cbs_bridge.cpp','cbs_bridge.cpp')]),
    ]
    for name,cwd,target,units in targets:
        directory=cwd/'CMakeFiles'/f'{target}.dir';baseflags=flags(directory/'flags.make')
        replacements={}
        for relative,overlay in units:
            obj=out/(overlay+'.o')
            run(['/usr/bin/c++']+extra+baseflags+sanitize+['-c',str(ROOT/'overlay'/overlay),'-o',str(obj)],cwd,name+'_'+overlay)
            replacements[str(Path('CMakeFiles')/f'{target}.dir'/(relative+'.o'))]=str(obj)
        link=shlex.split((directory/'link.txt').read_text())
        link=[replacements.get(x,x) for x in link]
        previous=link[link.index('-o')+1];dest=out/Path(previous).name
        link[link.index('-o')+1]=str(dest);link+=profile_link
        run(link,cwd,name+'_link')
        # Use recorded SONAME and conventional unversioned symlink, scoped here.
        soname=next((x.split(',')[-1] for x in link if x.startswith('-Wl,-soname,')),dest.name)
        for alias in {soname,f'lib{name}.so'}:
            if alias!=dest.name:(out/alias).symlink_to(dest.name)
    gpu_commands=json.loads((OLD/'gpu_attempt1_commands.json').read_text())
    compile=gpu_commands[1];obj=out/'integrated_vgicp_factor_gpu.cpp.o'
    compile=[str(ROOT/'overlay/integrated_vgicp_factor_gpu.cpp') if x.endswith('/factors/integrated_vgicp_factor_gpu.cpp') else x for x in compile]
    compile[compile.index('-o')+1]=str(obj);compile[1:1]=extra+sanitize
    cwd=WS/'build/gtsam_points'
    run(compile,cwd,'gpu_compile')
    link=gpu_commands[2]
    link=[str(obj) if x==str(OLD/'integrated_vgicp_factor_gpu.o') else x for x in link]
    dest=out/'libgtsam_points_cuda.so.1.2.1';link[link.index('-o')+1]=str(dest);link+=profile_link
    run(link,cwd,'gpu_link')
    for name in ['libgtsam_points_cuda.so','libgtsam_points_cuda.so.1']:(out/name).symlink_to(dest.name)
    # Keep old test command's include/ABI/link settings; only private library order changes.
    test_base=json.loads((OLD/'test_attempt3_commands.json').read_text())[0]
    for source in ('test_profile.cpp','test_solver.cpp','test_gpu.cpp'):
        cmd=[str(ROOT/source) if x.endswith('/cbs_local_graph_linearizer_test.cpp') else x for x in test_base]
        cmd[cmd.index('-o')+1]=str(out/Path(source).stem)
        cmd[1:1]=extra+sanitize+['-L'+str(out)]
        cmd+=profile_link+['-ltbb']
        run(cmd,ROOT,source)
    assert all(sha(p)==h for p,h in protected.items()),'protected build input changed'
    (out/'complete.json').write_text(json.dumps(dict(protected_unchanged=len(protected),
        artifacts={p.name:sha(p) for p in out.iterdir() if p.is_file() and not p.is_symlink() and p.suffix not in ('.log','.json')}),indent=2)+'\n')
    print('BUILD_COMPLETE',out,flush=True)

if __name__=='__main__':main()
