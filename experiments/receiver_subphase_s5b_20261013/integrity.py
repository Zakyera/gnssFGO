"""Preserve source artifacts and pin the diagnostic overlay; no estimator run."""
import difflib
import hashlib
import json
from pathlib import Path
import subprocess
import sys

HERE = Path(__file__).resolve().parent
ROOT = Path('/home/yeranis/results/gnssfgo_receiver_subphase_s5b_20261013')
PRIOR = HERE.parent/'shared_linearization_s5_live160_20261012/integrity.json'


def sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for b in iter(lambda:f.read(1<<20), b''): h.update(b)
    return h.hexdigest()


def main():
    phase=sys.argv[1]; assert phase in ['before','after']
    output=ROOT/(phase+'_integrity.json'); assert not output.exists()
    protected=json.loads((ROOT/'protected.json').read_text())
    for path,expected in protected.items(): assert sha(path)==expected,path
    prior=json.loads(PRIOR.read_text())
    repos={}
    for path,expected in prior['source_repositories'].items():
        def git(*args): return subprocess.check_output(['git','-C',path,*args],text=True).strip()
        current=dict(head=git('rev-parse','HEAD'),status=git('status','--porcelain'))
        assert current==expected,(path,current,expected)
        repos[path]=current
    for path,expected in prior['raw_artifacts'].items(): assert sha(path)==expected['sha256'],path
    files={str(p):sha(p) for p in (ROOT/'build').rglob('*') if p.is_file() and
      (p.suffix in ['.h','.cpp'] or p.name=='replay')}
    if phase=='after':
        before=json.loads((ROOT/'before_integrity.json').read_text())
        assert files==before['diagnostic_files'],'diagnostic files changed during profile'
    else:
        original=json.loads((ROOT/'preparation.json').read_text())['originals']
        diff=[]
        for path in original:
            p=Path(path)
            if '/online_fgo/include/' in path: relative=path.split('/online_fgo/include/')[1]
            else: relative=p.name
            target=ROOT/'build'/relative
            diff.extend(difflib.unified_diff(p.read_text().splitlines(True),target.read_text().splitlines(True),path,str(target)))
        with (ROOT/'frozen_instrumentation.diff').open('x') as f:f.writelines(diff)
    result=dict(passed=True,protected_files=len(protected),raw_artifacts=len(prior['raw_artifacts']),
      source_repositories=repos,diagnostic_files=files,
      current_head=subprocess.check_output(['git','-C',str(HERE.parents[1]),'rev-parse','HEAD'],text=True).strip())
    with output.open('x') as f:json.dump(result,f,indent=2)
    print(phase,'integrity passed',len(protected),'protected inputs;',len(files),'diagnostic files')


if __name__=='__main__': main()
