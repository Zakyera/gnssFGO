"""Read-only verification of saved text and original-byte reconstruction hashes."""
from pathlib import Path
import hashlib
import json

root = Path(__file__).resolve().parent
manifest = json.loads((root / 'ARCHIVE_MANIFEST.json').read_text())
for row in manifest:
    path = root / row['archive']
    assert path.resolve().is_relative_to(root)
    data = path.read_bytes()
    assert hashlib.sha256(data).hexdigest() == row['archived_sha256'], path
    if row['final_newline_added']:
        assert data.endswith(b'\n'), path
        data = data[:-1]
    assert len(data) == row['bytes'], path
    assert hashlib.sha256(data).hexdigest() == row['original_sha256'], path
print(f'PASS: {len(manifest)} archived files and reconstructable original hashes')
