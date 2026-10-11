"""Read-only integrity check; does not execute any archived experiment."""
import argparse
import hashlib
import json
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument('--check-local-originals', action='store_true')
args = parser.parse_args()
root = Path(__file__).resolve().parent
rows = json.loads((root / 'ARCHIVE_MANIFEST.json').read_text())
for row in rows:
    archived = root / row['archive']
    assert archived.resolve().is_relative_to(root), archived
    data = archived.read_bytes()
    assert len(data) == row['bytes'], archived
    assert hashlib.sha256(data).hexdigest() == row['sha256'], archived
    if args.check_local_originals:
        original = Path(row['original'])
        assert hashlib.sha256(original.read_bytes()).hexdigest() == row['sha256'], original
print(f"PASS: {len(rows)} byte-identical archive files; local originals checked={args.check_local_originals}")
