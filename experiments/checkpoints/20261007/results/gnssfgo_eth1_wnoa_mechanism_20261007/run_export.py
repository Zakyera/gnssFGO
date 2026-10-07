from pathlib import Path
import subprocess,json,sys,time
r=Path('/audit');action=sys.argv[1]
if action=='build':
    inc=['-I/runtime/include','-I/source/online_fgo','-I/source/online_fgo/include','-I/usr/include/eigen3']
    inc+=['-I'+str(p) for p in Path('/opt/ros/humble/include').iterdir() if p.is_dir()]
    command=['c++','-std=c++20','-O2','-DNDEBUG',*inc,'/audit/interaction.cpp','-L/runtime/lib','-lonline_fgo_solver','-lonline_fgo_util','/runtime/lib/libgtsam.so.4','-ltbb','-Wl,-rpath,/runtime/lib','-o','/audit/interaction_export']
else:
    assert action=='export';(r/'interaction').mkdir(exist_ok=False)
    command=['/audit/interaction_export','/audit/requests.csv']
t=time.monotonic()
with (r/f'{action}.log').open('x') as f:p=subprocess.run(command,stdout=f,stderr=subprocess.STDOUT)
(r/f'{action}_command.json').write_text(json.dumps(dict(command=command,returncode=p.returncode,seconds=time.monotonic()-t),indent=2))
print(action,p.returncode,flush=True);sys.exit(p.returncode)
