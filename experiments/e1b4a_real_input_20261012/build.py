"""Only a new private wrapper; retained GTSAM/CBS/GPU code is linked unchanged."""
from pathlib import Path
import json
import shlex
import subprocess
import time
import hashlib

WS=Path('/workspace/cbs_gtsam4.3')
ROOT=WS/'runs/e1b4a_real_input_20261012'
BATCH=WS/'runs/e1b4a_local_update_batching_20261012'
OLD=WS/'runs/e1b_local_profile_20261012'
def main():
    out=ROOT/'release_1';out.mkdir(exist_ok=False)
    for p in (BATCH/'release_2').glob('lib*.so*'):
        if p.name!='libglim_ros.so':(out/p.name).symlink_to(p.resolve())
    records=json.loads((BATCH/'release_2/libraries_commands.json').read_text())
    includes=['-I'+str(ROOT/'include'),'-I'+str(ROOT/'authoring')]
    commands=[]
    for name in ['glim_ros','glim_factor_graph_inspector','cbs_bridge','ExactGpuSnapshot']:
        r=next(r for r in records if Path(r['command'][-1]).name=='cbs_bridge.cpp.o' if '-c' in r['command']) if name=='ExactGpuSnapshot' else next(
          r for r in records if '-c' in r['command'] and Path(r['command'][-1]).name==name+'.cpp.o')
        cmd=list(r['command']);cmd[1:1]=includes
        if name in ['cbs_bridge','ExactGpuSnapshot']:cmd[cmd.index('-c')+1]=str(ROOT/'authoring'/(name+'.cpp'))
        cmd[cmd.index('-o')+1]=str(out/(name+'.cpp.o'))
        if name=='ExactGpuSnapshot':cmd+=['-I/usr/local/cuda/include']
        execute(cmd,r['cwd'],name,out,commands)
    r=next(r for r in records if '-o' in r['command'] and Path(r['command'][r['command'].index('-o')+1]).name=='libglim_ros.so')
    cmd=[str(out/Path(x).name) if x.startswith(str(BATCH/'release_2')) and x.endswith('.cpp.o') else x for x in r['command']]
    cmd[cmd.index('-o')+1]=str(out/'libglim_ros.so')
    cmd += [str(out/'ExactGpuSnapshot.cpp.o'),'-L/usr/local/cuda/lib64','-lcudart']
    execute(cmd,r['cwd'],'link',out,commands)
    (out/'complete.json').write_text(json.dumps(dict(passed=True,sha256=hashlib.sha256((out/'libglim_ros.so').read_bytes()).hexdigest()),indent=2))
def execute(cmd,cwd,name,out,commands):
    start=time.monotonic()
    with (out/(name+'.log')).open('x') as log:r=subprocess.run(cmd,cwd=cwd,stdout=log,stderr=subprocess.STDOUT)
    commands.append(dict(command=cmd,cwd=cwd,returncode=r.returncode,seconds=time.monotonic()-start))
    (out/'commands.json').write_text(json.dumps(commands,indent=2))
    print(name,r.returncode,commands[-1]['seconds'],flush=True)
    if r.returncode:print((out/(name+'.log')).read_text()[-7000:]);raise SystemExit(r.returncode)
if __name__=='__main__':main()
