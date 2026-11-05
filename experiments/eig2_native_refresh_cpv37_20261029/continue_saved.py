"""Continue only never-started saved checkpoints after dependency-only repair."""
import ast
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import time

H=Path(__file__).resolve().parent
# Load only definitions/constants, not the earlier controller's executable body.
tree=ast.parse((H/'run_ablation.py').read_text())
allowed=[]
for node in tree.body:
    if isinstance(node,(ast.Import,ast.ImportFrom,ast.FunctionDef)):
        allowed.append(node)
    elif isinstance(node,ast.Assign) and all(isinstance(t,ast.Name) and t.id in {'H','W','OLD','OH','P','Q','OQ','CONTAINER'} for t in node.targets):
        allowed.append(node)
namespace={'__file__':str(H/'run_ablation.py')}
exec(compile(ast.Module(body=allowed,type_ignores=[]),str(H/'run_ablation.py'),'exec'),namespace)
for key in ['W','OLD','OH','P','Q','OQ','CONTAINER','sha','save','check','command','container_states','env_args']:
    globals()[key]=namespace[key]

assert not (H/'continuation_start.json').exists()
assert not (P/'results').exists(), 'No dataset branch may have run'
assert container_states()==['/cbsms_ws_gpu false','/ubuntu-jammy-humble-nvidia false']
assert json.loads((H/'restoration.json').read_text())['comparison_completed'] is False
pins=json.loads((H/'protected_before.json').read_text())
check(pins)
check(json.loads((H/'implementation_before.json').read_text()))
for v in ['release','sanitizer']:
    assert json.loads((P/('build_'+v)/'passed.json').read_text())['passed']
    for suffix in ['','_displaced']:
        root=P/('build_'+v)/('fixture'+suffix)
        assert sha(root/'input_before.bin')==sha(root/'input_after.bin')
assert shutil.disk_usage(W).free>8*1024**3
save(H/'continuation_start.json',dict(pid=os.getpid(),wall_ns=time.time_ns(),source_runs=0))
completed=False
try:
    subprocess.run(['docker','start',CONTAINER],check=True)
    command(env_args()+['readelf','-d',Q+'/build_release/test'],H/'elf_dependencies.log')
    needed=(H/'elf_dependencies.log').read_text()
    assert 'Shared library: [libgtsam.so.4]' in needed
    assert 'Shared library: [libgtsam_unstable.so.4]' in needed
    assert 'Shared library: [libcbs.so]' not in needed
    command(env_args()+['ldd',Q+'/build_release/test'],H/'actual_dependencies.log')
    text=(H/'actual_dependencies.log').read_text()
    assert 'not found' not in text
    for lib in ['libgtsam.so.4','libgtsam_unstable.so.4']:
        assert lib+' => /workspace/cbs_gtsam4.3/devel/lib/'+lib+' ' in text
        path=str(W/'devel/lib'/lib)
        assert path in pins and sha(path)==pins[path]
    save(H/'ready.json',dict(passed=True,release=True,sanitizer=True,
         executable_sha256=sha(P/'build_release/test'),actual_gtsam_dependencies_pinned=True))
    for k in [800,934,1000,1335,1336,1337,1338]:
        assert shutil.disk_usage(W).free>5*1024**3
        command(env_args()+[Q+'/build_release/test','data',OQ+'/capture/snapshots/update_'+str(k)+'/checkpoint.bin',Q+'/results/update_'+str(k)],H/('checkpoint_'+str(k)+'.log'))
        root=P/'results'/('update_'+str(k))
        assert sha(root/'input_before.bin')==sha(root/'input_after.bin')
    check(pins)
    completed=True
    save(H/'finished.json',dict(passed=True,comparison_completed=True,source_runs=0,outcomes_require_interpretation=True))
except Exception as e:
    save(H/'continuation_failed.json',dict(error=repr(e),source_runs=0))
    raise
finally:
    top=subprocess.check_output(['docker','top',CONTAINER,'-eo','pid,comm'],text=True)
    assert all(line.split()[-1]=='tail' for line in top.splitlines()[1:]),top
    subprocess.run(['docker','stop','-t','20',CONTAINER],check=True)
    states=container_states()
    save(H/'continuation_restoration.json',dict(states=states,top=top,comparison_completed=completed))
    assert states==['/cbsms_ws_gpu false','/ubuntu-jammy-humble-nvidia false']
