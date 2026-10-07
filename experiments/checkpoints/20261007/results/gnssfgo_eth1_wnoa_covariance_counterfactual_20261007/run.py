from pathlib import Path
import subprocess,json,sys,time
r=Path('/audit');action=sys.argv[1]
if action=='build':
    inc=['-I/previous/build/wnoa','-I/runtime/include','-I/source/online_fgo','-I/source/online_fgo/include','-I/usr/include/eigen3']
    inc+=['-I'+str(p) for p in Path('/opt/ros/humble/include').iterdir() if p.is_dir()]
    cmd=['c++','-std=c++20','-O2','-DNDEBUG',*inc,'/audit/counterfactual.cpp','-L/runtime/lib','-lonline_fgo_solver','-lonline_fgo_util','/runtime/lib/libgtsam.so.4','-ltbb','-Wl,-rpath,/runtime/lib','-o','/audit/counterfactual']
else:
    assert action=='run' and not (r/'unchanged_control').exists() and not (r/'on_means_off_cov').exists()
    cmd=['/audit/counterfactual']
start=time.monotonic()
with (r/f'{action}.log').open('x') as f:p=subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT)
with (r/f'{action}_command.json').open('x') as f:json.dump(dict(command=cmd,returncode=p.returncode,seconds=time.monotonic()-start),f,indent=2)
print(action,p.returncode,flush=True);sys.exit(p.returncode)
