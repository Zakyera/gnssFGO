"""Recheck original inputs/source repositories, no estimator execution."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent.parent/'receiver_subphase_s5b_20261013'))
# This module name may match the running script, so import by file explicitly.
import importlib.util
spec=importlib.util.spec_from_file_location('s5b_integrity',Path(__file__).resolve().parent.parent/'receiver_subphase_s5b_20261013/integrity.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
import json,subprocess
ROOT=Path('/home/yeranis/results/gnssfgo_receiver_sampling_s5c_20261013')
phase=sys.argv[1];assert phase in ['before','after']
output=ROOT/(phase+'_integrity.json');assert not output.exists()
protected=json.loads((m.ROOT/'protected.json').read_text())
for path,want in protected.items():assert m.sha(path)==want,path
prior=json.loads(m.PRIOR.read_text());repos={}
for path,want in prior['source_repositories'].items():
    def git(*args):return subprocess.check_output(['git','-C',path,*args],text=True).strip()
    got=dict(head=git('rev-parse','HEAD'),status=git('status','--porcelain'));assert got==want,path
    repos[path]=got
for path,want in prior['raw_artifacts'].items():assert m.sha(path)==want['sha256'],path
result=dict(passed=True,protected_files=len(protected),raw_artifacts=len(prior['raw_artifacts']),source_repositories=repos,
    sampler_sha256=m.sha(ROOT/'profiler/libreceiver_sampler.so'))
result['deep_sampler_sha256']=m.sha(ROOT/'profiler/libreceiver_sampler_deep.so')
assert result['deep_sampler_sha256']==json.loads((ROOT/'fixture_deep.json').read_text())['collector_sha256']
if phase=='after':
    result['deep8_sampler_sha256']=m.sha(ROOT/'profiler/libreceiver_sampler_deep8.so')
    assert result['deep8_sampler_sha256']==json.loads((ROOT/'fixture_deep8.json').read_text())['collector_sha256']
if phase=='after':assert json.loads((ROOT/'before_integrity.json').read_text())['sampler_sha256']==result['sampler_sha256']
with output.open('x') as f:json.dump(result,f,indent=2)
print(phase,'PASS',result['protected_files'],result['raw_artifacts'])
