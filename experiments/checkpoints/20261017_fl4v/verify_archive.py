#!/usr/bin/env python3
"""Read-only checkpoint validation; no imports of experiment controllers."""
import argparse
import ast
import hashlib
import json
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

parser = argparse.ArgumentParser()
parser.add_argument('--originals', action='store_true', help='also verify original host files/binaries')
args = parser.parse_args()
root = Path(__file__).resolve().parent
manifest = json.loads((root / 'manifest.json').read_text())


def digest(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


counts = dict(files=0, python=0, shell=0, json=0, xml=0, jsonc_hash_only=0)
for entry in manifest['files']:
    path = root / entry['archived']
    assert digest(path) == entry['sha256'], path
    assert path.stat().st_size == entry['size'], path
    if args.originals:
        assert digest(Path(entry['original'])) == entry['sha256'], entry['original']
    counts['files'] += 1
    if path.suffix == '.py':
        ast.parse(path.read_text(), filename=str(path))
        counts['python'] += 1
    elif path.suffix in {'.sh', '.env'}:
        subprocess.run(['bash', '-n', str(path)], check=True)
        counts['shell'] += 1
    elif path.suffix == '.json':
        # GLIM intentionally accepts comments in its configuration files.
        # Preserve those exact bytes; do not rewrite them into strict JSON.
        if 'glim_config' in path.parts:
            counts['jsonc_hash_only'] += 1
        else:
            json.loads(path.read_text())
            counts['json'] += 1
    elif path.suffix == '.launch':
        ET.parse(path)
        counts['xml'] += 1
if args.originals:
    for entry in manifest['local_binaries']:
        assert digest(Path(entry['resolved'])) == entry['sha256'], entry['resolved']
print(json.dumps(dict(passed=True, originals_checked=args.originals, **counts)))
