"""Read-only hashes and source-repository identity; exclusive new evidence files."""
import importlib.util
import json
from pathlib import Path
import subprocess
import sys

HERE=Path(__file__).resolve().parent
ROOT=Path('/home/yeranis/results/gnssfgo_endpoint_reuse_e2b_20261013')
REF=Path('/home/yeranis/results/gnssfgo_receiver_sampling_s5c_20261013')
spec=importlib.util.spec_from_file_location('s5b_integrity',HERE.parent/'receiver_subphase_s5b_20261013/integrity.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
phase=sys.argv[1];assert phase in ['before','after']
out=ROOT/(phase+'_integrity.json');assert not out.exists()
protected=json.loads((m.ROOT/'protected.json').read_text())
for path,want in protected.items():assert m.sha(path)==want,path
prior=json.loads(m.PRIOR.read_text());repos={}
for path,want in prior['source_repositories'].items():
    def git(*args):return subprocess.check_output(['git','-C',path,*args],text=True).strip()
    got=dict(head=git('rev-parse','HEAD'),status=git('status','--porcelain'));assert got==want,(path,got,want)
    repos[path]=got
for path,want in prior['raw_artifacts'].items():assert m.sha(path)==want['sha256'],path
reference={str(p):m.sha(p) for folder in ['reference','prefix_reference'] for p in (REF/folder).iterdir() if p.is_file()}
result=dict(passed=True,protected_files=len(protected),raw_artifacts=len(prior['raw_artifacts']),
            source_repositories=repos,reference=reference)
if phase=='after':assert json.loads((ROOT/'before_integrity.json').read_text())['reference']==reference
with out.open('x') as f:json.dump(result,f,indent=2)
print(phase,'PASS',len(protected),len(prior['raw_artifacts']),len(reference))
