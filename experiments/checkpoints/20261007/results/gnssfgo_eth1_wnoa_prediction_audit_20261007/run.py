from pathlib import Path
import subprocess,sys,json,time
R=Path('/audit');label=sys.argv[1];records=[]
incs=['-I/runtime/include','-I/source/online_fgo','-I/source/online_fgo/include','-I/usr/include/eigen3']
incs+=['-I'+str(p) for p in Path('/opt/ros/humble/include').iterdir() if p.is_dir()]
link=['-L/runtime/lib','-lonline_fgo_solver','-lonline_fgo_util','/runtime/lib/libgtsam.so.4','-ltbb','-Wl,-rpath,/runtime/lib']
def run(name,argv):
 t=time.monotonic()
 with (R/f'{label}_{name}.log').open('x') as f:r=subprocess.run(argv,stdout=f,stderr=subprocess.STDOUT)
 records.append(dict(name=name,argv=argv,seconds=time.monotonic()-t,returncode=r.returncode))
 (R/f'commands_{label}.json').write_text(json.dumps(records,indent=2)+'\n')
 print({k:v for k,v in records[-1].items() if k!='argv'},flush=True)
 if r.returncode:print((R/f'{label}_{name}.log').read_text());raise SystemExit(r.returncode)
if label=='build':run('compile',['c++','-std=c++20','-O2','-DNDEBUG',*incs,'/audit/audit.cpp',*link,'-o','/audit/audit'])
elif label=='means':run('export',['/audit/audit','means','/audit/means','/audit/publications.csv'])
elif label=='means_repeat':run('export',['/audit/audit','means','/audit/means_repeat','/audit/publications.csv'])
elif label=='velocity':
 run('compile',['c++','-std=c++20','-O2','-DNDEBUG',*incs,'/audit/velocity_check.cpp',*link,'-o','/audit/velocity_check'])
 run('export',['/audit/velocity_check','/audit/velocity_check.csv'])
elif label in ['primary','repeat']:run('export',['/audit/audit','covariance','/audit/'+label,'/audit/requests.csv'])
elif label=='sanitizer':
 run('compile',['c++','-std=c++20','-O1','-g1','-DEIGEN_MALLOC_ALREADY_ALIGNED=1','-fsanitize=address,undefined','-fno-omit-frame-pointer',*incs,'/audit/audit.cpp',*link,'-o','/audit/audit_sanitized'])
 run('test',['/audit/audit_sanitized','--test'])
 run('export',['/audit/audit_sanitized','covariance','/audit/sanitized','/audit/sanitizer_requests.csv'])
