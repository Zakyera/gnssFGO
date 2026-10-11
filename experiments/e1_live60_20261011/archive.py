#!/usr/bin/env python3
"""Save small exact live-acceptance evidence; never copy bags or executables."""
import hashlib
import json
from pathlib import Path
import shutil

HERE=Path(__file__).resolve().parent
RUN=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_e1_live60_20261011')
ROOT=RUN/'prepared'
DEST=HERE/'evidence'
assert json.loads((ROOT/'live60_on_validation.json').read_text())['passed']
assert json.loads((RUN/'engineering_summary.json').read_text())['passed_validation']
DEST.mkdir(exist_ok=False)
files=[p for p in RUN.glob('*.py')]+[RUN/'engineering_summary.json',RUN/'resource_samples.jsonl']
files+=[p for p in ROOT.iterdir() if p.is_file() and p.suffix in ['.py','.json','.sh','.env','.launch']]
files+=list((ROOT/'preflight').glob('*'))
files+=list((RUN/'validation_logs').glob('*'))
source=ROOT/'live60_on_source'
for name in ['run_metadata.txt','source_coverage.json','effective_mode_check.json',
             'resolved_glim_params.yaml','resolved_kimera_params.yaml']:
    files.append(source/name)
for tree in ['glim_config','kimera_profile']:
    files.extend(p for p in (source/tree).rglob('*') if p.is_file())
files+=list((ROOT/'live60_on_control').glob('*.json'))
files+=[ROOT/'live60_on_control/monitor.jsonl',ROOT/'live60_on_control/source.log']
receiver=ROOT/'live60_on_receiver'
for name in ['manifest.json','resolved_config.json','transport_status.json']:
    files.append(receiver/name)
for name in ['summary.json','source_accounting.csv','unsupported_beliefs.csv','publication_convergence.csv']:
    files.append(receiver/'receiver'/name)
manifest={};omitted=[]
for p in sorted(set(files)):
    assert p.is_file(),p
    if p.stat().st_size>2_000_000:
        omitted.append({'path':str(p),'bytes':p.stat().st_size,'reason':'large retained local artifact'});continue
    target=DEST/p.relative_to(RUN);target.parent.mkdir(parents=True,exist_ok=True)
    shutil.copy2(p,target)
    digest=hashlib.sha256(p.read_bytes()).hexdigest()
    assert digest==hashlib.sha256(target.read_bytes()).hexdigest()
    manifest[str(target.relative_to(HERE))]={'original':str(p),'bytes':p.stat().st_size,'sha256':digest}
result={'copied':manifest,'omitted':omitted,'run_root':str(RUN),
        'note':'Large matrices/logs/bags/journals/binaries remain local; protected and validation manifests retain hashes.'}
with (HERE/'ARCHIVE_MANIFEST.json').open('x') as f:json.dump(result,f,indent=2)
print('Archived',len(manifest),'files;',sum(v['bytes'] for v in manifest.values()),'bytes; omitted',len(omitted))
