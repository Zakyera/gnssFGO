"""Relink identical sanitizer objects non-PIE; preserve the failed PIE launch."""
from pathlib import Path
import json,subprocess
ROOT=Path('/workspace/cbs_gtsam4.3/runs/shared_linearization_s4_provider_20261012')
old=ROOT/'asan_1';out=ROOT/'asan_nopie';out.mkdir(exist_ok=False)
cmd=next(r['command'] for r in json.loads((old/'commands.json').read_text())
         if '-o' in r['command'] and str(old/'test_shadow') in r['command'])
cmd=[str(out/'test_shadow') if x==str(old/'test_shadow') else x for x in cmd]+['-no-pie']
(out/'command.json').write_text(json.dumps(cmd,indent=2))
with (out/'link.log').open('x') as log:subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT,check=True)
for name in ['libgtsam_points.so.1.2.1','libgtsam_points.so.1','libgtsam_points.so']:
    (out/name).symlink_to(old/name)
subprocess.run(['python3',str(ROOT/'driver/run_tests.py'),str(out),'tests'],check=True)
