"""Read-only production integrity checks; only the named new evidence file is written."""
from pathlib import Path
import hashlib
import json
import subprocess
import sys

REPO = Path('/home/yeranis/repos/gnssFGO')
WS = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')

def sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda:stream.read(1024*1024),b''):h.update(chunk)
    return h.hexdigest()

def main():
    out=Path(sys.argv[1])
    if out.exists():raise SystemExit('refuse evidence overwrite')
    manifest=json.loads((REPO/'experiments/shared_linearization_b0_20261012/evidence/protected_manifest.json').read_text())
    baseline=json.loads((REPO/'experiments/shared_linearization_s1_20261012/INTEGRITY.json').read_text())
    failures=[]
    for path,digest in manifest.items():
        if sha(path)!=digest:failures.append(path)
    for path,item in baseline['inspected_evidence'].items():
        if sha(WS/path)!=item['sha256']:failures.append(path)
    repositories={}
    for path,item in baseline['source_repositories'].items():
        head=subprocess.check_output(['git','-C',path,'rev-parse','HEAD'],text=True).strip()
        status=subprocess.check_output(['git','-C',path,'status','--porcelain'],text=True)
        repositories[path]={'head':head,'status':status}
        if head!=item['head'] or status:failures.append(path)
    cpu=WS/'devel/lib/libgtsam_points.so.1.2.1'
    if sha(cpu)!='a1fac7bede8bf7dafef2f3ae3d7206fd539b0285479d8a6cb739f8da22b0f593':failures.append(str(cpu))
    result={'protected_files':len(manifest),'S1_pins':len(baseline['inspected_evidence']),
            'source_repositories':repositories,'main_cpu_library_sha256':sha(cpu),'failures':failures}
    with out.open('x') as stream:json.dump(result,stream,indent=2)
    print(json.dumps({'protected_files':len(manifest),'failures':failures}))
    raise SystemExit(bool(failures))
if __name__=='__main__':main()
