"""Read/hash frozen inputs and record verification; never modifies them."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys

ROOT=Path(__file__).resolve().parent
REPO=Path('/home/yeranis/repos/gnssFGO')
FROZEN=Path('/home/yeranis/results/gnssfgo_eth1_local_prediction_20261007')
CAPTURE=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_eth1_covariance160_validated_20261010')
def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
    return h.hexdigest()

label=sys.argv[1]
protected=json.loads((CAPTURE/'prepared/protected_manifest.json').read_text())
protected.update(json.loads((CAPTURE/'additional_protected_evidence.json').read_text()))
verification={p:dict(expected=h,actual=sha(p)) for p,h in protected.items()}
helpers=json.loads((FROZEN/'source_provenance.json').read_text())['source_sha256']
source={p:dict(expected=h,actual=sha(REPO/p)) for p,h in helpers.items()}
original_outputs={str(p):sha(p) for p in (CAPTURE/'prepared/full160_on_receiver/receiver').iterdir() if p.is_file()}
journal=CAPTURE/'prepared/full160_on_receiver/events.csv'
result=dict(protected=verification,helpers=source,original_outputs=original_outputs,
            journal={str(journal):sha(journal)},
            repo_head=subprocess.check_output(['git','-C',str(REPO),'rev-parse','HEAD'],text=True).strip(),
            repo_status=subprocess.check_output(['git','-C',str(REPO),'status','--short'],text=True),
            private_files={str(p.relative_to(ROOT)):sha(p) for p in (ROOT/'build').rglob('*') if p.is_file()})
bad=[p for data in (verification,source) for p,x in data.items() if x['expected']!=x['actual']]
result['mismatches']=bad
with (ROOT/f'provenance_{label}.json').open('x') as f:json.dump(result,f,indent=2)
print(json.dumps(dict(protected=len(verification),helpers=len(source),mismatches=bad,journal=result['journal']),indent=2))
assert not bad
