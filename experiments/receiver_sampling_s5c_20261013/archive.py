"""Retain compact audit evidence; raw samples/binaries stay in the result root."""
from pathlib import Path
import hashlib,json,shutil
from symbolize import ROOT
HERE=Path(__file__).resolve().parent
output=HERE/'evidence';assert not output.exists();output.mkdir()
names=['before_integrity.json','after_integrity.json','reference.json',
       'reference_validation.json','offline_baseline_identity.json',
       'reference_timings.json','endpoint_reuse_opportunity.json','prefix_validation.json']
for tag in ['fixture','fixture_founder','fixture_unwind','fixture_deep','fixture_deep8',
            'prefix_reference','prefix_sample','prefix_deep','prefix_deep8',
            'replay_sample','replay_sample20_deep8']:
    for suffix in ['.json','.log','_validation.json','_sampling_validation.json','_timings.json']:
        if (ROOT/(tag+suffix)).is_file():names.append(tag+suffix)
names+=['replay_sample_censored_attribution.json','replay_sample20_deep8_attribution.json']
for name in names:shutil.copy2(ROOT/name,output/name)
hashes={}
for path in sorted(ROOT.rglob('*')):
    if path.is_file() and path.name!='controller.lock':
        h=hashlib.sha256()
        with path.open('rb') as f:
            for block in iter(lambda:f.read(1<<20),b''):h.update(block)
        hashes[str(path)]=dict(bytes=path.stat().st_size,sha256=h.hexdigest())
with (output/'artifact_manifest.json').open('x') as f:json.dump(hashes,f,indent=2)
print('Archived',len(names),'compact artifacts;',len(hashes),'files hashed')
