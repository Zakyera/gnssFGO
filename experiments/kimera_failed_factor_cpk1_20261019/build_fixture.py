import json,os,subprocess
from pathlib import Path
R=Path('/workspace/cbs_gtsam4.3/runs/kimera_failed_factor_cpk1_20261019');A=R/'analysis'
c=json.loads((R/'actual_test_commands.json').read_text());env=os.environ.copy()
env['LD_LIBRARY_PATH']=str(R/'private_build')+':'+env.get('LD_LIBRARY_PATH','')
mapping={str(R/'test.cpp'):str(A/'smart_fixture.cpp'),str(R/'test.o'):str(A/'smart_fixture.o'),str(R/'test'):str(A/'smart_fixture')}
for name,cmd in [('compile',[mapping.get(a,a) for a in c['compile']]),('link',[mapping.get(a,a) for a in c['link']]),('run',[str(A/'smart_fixture')])]:
    with (A/('fixture_'+name+'.log')).open('x') as log:p=subprocess.run(cmd,env=env,stdout=log,stderr=subprocess.STDOUT)
    print(name,p.returncode,flush=True)
    if p.returncode:raise SystemExit(p.returncode)
