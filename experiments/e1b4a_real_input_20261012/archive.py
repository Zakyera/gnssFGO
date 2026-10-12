"""Retain small diagnostic evidence in git; leave recordings/builds local."""
from pathlib import Path
import json
import shutil
from prepare import ROOT,HERE,sha

out=HERE/'evidence';out.mkdir(exist_ok=False)
selected=['preflight.json','validation.json','control/complete.json','control/failed.txt',
          'control/source.log','release_1/complete.json','release_1/commands.json',
          'regression.xml','ldd.log','audit/FAILED.txt','audit/updates.jsonl',
          'e1_source/source_coverage.json','e1_source/effective_mode_check.json',
          'e1_source/run_metadata.txt','e1_source/resolved_glim_params.yaml',
          'e1_source/resolved_kimera_params.yaml','protected.json','source_git.json']
selected += [str(p.relative_to(ROOT)) for p in (ROOT/'audit').glob('sample_*.json')]
manifest={}
for name in selected:
    p=ROOT/name
    if not p.exists():continue
    assert p.stat().st_size<3*1024**2,name
    dest=out/name;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(p,dest)
    manifest[name]=dict(source=str(p),sha256=sha(p),bytes=p.stat().st_size)
(out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
print(len(manifest),'artifacts archived')
