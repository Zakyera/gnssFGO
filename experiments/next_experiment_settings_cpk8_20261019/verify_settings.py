#!/usr/bin/env python3
"""Read-only settings/hash check. Never starts a container, build or experiment."""
import argparse
import hashlib
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--require-ready', action='store_true')
    args = parser.parse_args()
    contract = json.loads(Path(__file__).with_name('settings.lock.json').read_text())
    expected = dict(contract['input_file_sha256'])
    for profile in contract['profiles'].values():
        root = Path(profile['source_directory'])
        for name, digest in profile['file_sha256'].items():
            expected[str(root / name)] = digest
    for libraries in contract['historical_loaded_libraries'].values():
        for library in libraries:
            expected[library['host_path']] = library['sha256']
    failures = []
    for name, digest in expected.items():
        path = Path(name)
        if not path.is_file():
            failures.append({'path': name, 'reason': 'missing'})
        elif hashlib.sha256(path.read_bytes()).hexdigest() != digest:
            failures.append({'path': name, 'reason': 'hash_changed'})
    print(json.dumps({'files_checked': len(expected), 'failures': failures,
                      'status': contract['status'],
                      'launch_allowed': contract['launch_allowed'],
                      'blocker': contract['blocker']}, indent=2))
    if failures:
        return 1
    if args.require_ready and (not contract['launch_allowed'] or
                               contract['kimera']['full_run_backend_selector'] is None):
        return 2
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
