import json,os,subprocess,time
from pathlib import Path
R=Path(__file__).resolve().parent
env=os.environ.copy();env['LD_LIBRARY_PATH']=str(R/'private_build')+':'+env.get('LD_LIBRARY_PATH','')
def run(args,name,cwd=None):
    name=os.environ.get('FL4P_BUILD_TAG','')+name
    with (R/(name+'.log')).open('x') as log:p=subprocess.run(args,cwd=cwd,env=env,stdout=log,stderr=subprocess.STDOUT)
    print(name,p.returncode,flush=True)
    if p.returncode:raise SystemExit(p.returncode)
t=json.loads((R/'actual_test_commands.json').read_text())
run(t['compile'],'test_compile');run(t['link'],'test_link');run([str(R/'test')],'test_run')
(R/'test_passed.json').write_text(json.dumps({'passed':True,'time':time.time()}))
c=json.loads((R/'build_commands.json').read_text())
run(c['compile'],'build_compile',c['cwd']);run(c['link'],'build_link',c['cwd'])
