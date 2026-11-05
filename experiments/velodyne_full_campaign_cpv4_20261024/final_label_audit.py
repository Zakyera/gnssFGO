"""Prelaunch literal reporting-label repair; numerical settings remain untouched."""
from pathlib import Path
import difflib, hashlib, json
H=Path(__file__).resolve().parent
assert not (H/'launch.json').exists()
receipt=[]
def sha(path):
 with path.open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
for slug in ('spx3','eig2','leica2','arc5'):
 out=H/slug
 info=json.loads((out/'prepared.json').read_text())
 P=Path(info['prepared']);manifest_path=P/'protected_manifest.json'
 assert not (out/'start.json').exists() and not any(P.glob('full_*_source'))
 manifest=json.loads(manifest_path.read_text());diff=[]
 for path in (P/'source_runner.sh',P/'source_runner_on.sh',P/'validate_case.py',out/'summarize.py'):
  before=sha(path);assert before==manifest[str(path)]
  old=path.read_text();assert old.count('bounded30.0')==1,path
  new=old.replace('bounded30.0','full-usable'+str(info['window']['duration_s']))
  # Change only the stale printed description, never a numeric literal/guard.
  path.write_text(new);manifest[str(path)]=sha(path)
  receipt.append(dict(path=str(path),old_sha256=before,new_sha256=sha(path)))
  diff.extend(difflib.unified_diff(old.splitlines(True),new.splitlines(True),fromfile=str(path)+' before-label-review',tofile=str(path)))
 path=out/'harness.diff';assert sha(path)==manifest[str(path)]
 with path.open('a') as f:f.write(''.join(diff))
 manifest[str(path)]=sha(path)
 manifest_path.write_text(json.dumps(manifest,indent=2))
with (H/'prelaunch_label_changes.json').open('x') as f:json.dump(receipt,f,indent=2)
print('Corrected 16 printed duration labels before any capture; all numerical settings unchanged.')
