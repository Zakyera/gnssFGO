"""Retain the failed PIE fixture attempt; relink identical objects non-PIE."""
from pathlib import Path
import hashlib
import json
import subprocess
ROOT=Path(__file__).resolve().parent.parent
old=ROOT/'asan';out=ROOT/'asan_nopie';out.mkdir(exist_ok=False)
commands=[]
with (out/'build.log').open('x') as log:
    for record in json.loads((old/'commands.json').read_text()):
        cmd=record['command']
        if '-c' in cmd:continue
        cmd=cmd.copy();index=cmd.index('-o')+1;cmd[index]=str(out/Path(cmd[index]).name);cmd.append('-no-pie')
        subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT,check=True);commands.append(cmd)
(out/'commands.json').write_text(json.dumps(commands,indent=2))
(out/'build_result.json').write_text(json.dumps(dict(passed=True,only_link_change='-no-pie',objects={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in old.glob('*.o')},binaries={n:hashlib.sha256((out/n).read_bytes()).hexdigest() for n in ['original','instrumented','tests']}),indent=2))
print('sanitizer objects reused unchanged, private non-PIE executables linked')
