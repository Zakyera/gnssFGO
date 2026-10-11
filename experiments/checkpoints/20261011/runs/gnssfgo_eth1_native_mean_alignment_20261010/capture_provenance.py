from pathlib import Path
import hashlib
import json
import subprocess

root=Path(__file__).resolve().parent
ws=root.parents[1]
def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        for b in iter(lambda:f.read(1048576),b''):h.update(b)
    return h.hexdigest()
manifest=json.loads((root/'arrival_boundary/protected_manifest.json').read_text())
assert all(sha(p)==v for p,v in manifest.items())
repos={}
for repo in [Path('/home/yeranis/repos/gnssFGO')]+[ws/'src'/n for n in ['Kimera-VIO','Kimera-VIO-ROS','glim','glim_ros1','cbs','cbsms']]:
    def git(*args):return subprocess.check_output(['git','-C',str(repo),*args],text=True)
    subprocess.run(['git','-C',str(repo),'diff','--check'],check=True)
    changed=git('diff','--name-only','HEAD').splitlines()+git('ls-files','--others','--exclude-standard').splitlines()
    repos[str(repo)]=dict(head=git('rev-parse','HEAD').strip(),status=git('status','--short'),
        changed_file_hashes={p:sha(repo/p) for p in changed if (repo/p).is_file()})
result=dict(status='IMPLEMENTED_PARTIALLY_VALIDATED_GPU_BLOCKED',
    covariances_modified=False,production_defaults_changed=False,
    native_kimera_gate_passed=True,native_glim_gate_passed=False,
    full40_run_started=False,protected_manifest_entries_unchanged=len(manifest),repos=repos)
with (root/'final_provenance.json').open('x') as f:json.dump(result,f,indent=2)
print(json.dumps({k:v for k,v in result.items() if k!='repos'},indent=2))
