"""One verbose focused-unit diagnostic after an ASan startup failure."""
from pathlib import Path
import os
import subprocess
import json
root=Path(__file__).resolve().parent
ws=root.parents[1]
out=root/'unit_glim_portable'
env=os.environ.copy()
env['LD_LIBRARY_PATH']=str(ws/'devel')+':'+str(ws/'devel/lib')+':'+env.get('LD_LIBRARY_PATH','')
env['ASAN_OPTIONS']='detect_leaks=0:verbosity=1'
env['UBSAN_OPTIONS']='halt_on_error=1:print_stacktrace=1'
with (out/'asan_verbose_diagnostic.log').open('x') as f:
    run=subprocess.run([str(out/'asan_ubsan')],env=env,stdout=f,stderr=subprocess.STDOUT)
with (out/'asan_verbose_diagnostic.json').open('x') as f:
    json.dump(dict(exit_code=run.returncode,prior_failure_preserved=True,
        scope='Focused helper unit only; no sensors or estimator. Linked native libraries are not instrumented.'),f,indent=2)
print(run.returncode)
