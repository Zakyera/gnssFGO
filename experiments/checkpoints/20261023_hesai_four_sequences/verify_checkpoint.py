"""Read-only local checkpoint integrity; --originals also checks copied originals."""
from pathlib import Path
import argparse,hashlib,json
p=argparse.ArgumentParser();p.add_argument('--originals',action='store_true');args=p.parse_args()
H=Path(__file__).resolve().parent
m=json.loads((H/'manifest.json').read_text())
for name,row in m['files'].items():
    paths=[H/name]+([Path(row['original'])] if args.originals else [])
    for path in paths:
        assert path.stat().st_size==row['bytes'],path
        with path.open('rb') as f:assert hashlib.file_digest(f,'sha256').hexdigest()==row['sha256'],path
print(f"PASS: {len(m['cases'])} cases, {len(m['files'])} saved files; originals={args.originals}")
