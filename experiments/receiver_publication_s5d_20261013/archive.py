"""Archive small observer evidence and hashes without changing any raw result."""
import difflib
import hashlib
import json
from pathlib import Path
import shutil

HERE = Path(__file__).resolve().parent
ROOT = Path('/home/yeranis/results/gnssfgo_publication_s5d_20261013')
E2B = HERE.parent / 'receiver_endpoint_reuse_e2b_20261013'
ORIGINAL = Path('/home/yeranis/results/gnssfgo_publication_cache_20261010/build')
for action in ['before', 'unit', 'sanitize', 'prefix_off', 'prefix_on', 'full_on', 'after']:
    assert json.loads((ROOT / (action + '.json')).read_text())['passed'], action
assert (ROOT / 'analysis.json').exists()
OUT = HERE / 'evidence'
OUT.mkdir()
for path in ROOT.iterdir():
    if path.suffix in ['.json', '.xml', '.log']:
        shutil.copy2(path, OUT / path.name)
patch = []
for name, base in [('DiagnosticReplay.cpp', E2B),
                   ('SourceWnoaModel.h', E2B), ('SnapshotAudit.h', E2B),
                   ('FullPublicationRefinement.h', ORIGINAL),
                   ('CachedGaussianEvaluation.h', ORIGINAL),
                   ('offline_process/BoundedISAM2Refinement.h',
                    HERE.parents[1] / 'online_fgo/include')]:
    patch.extend(difflib.unified_diff((base / name).read_text().splitlines(True),
                 (HERE / name).read_text().splitlines(True), str(base / name), str(HERE / name)))
with (OUT / 'private_changes.diff').open('x') as f:
    f.writelines(patch)
manifest = {}
for path in sorted(ROOT.rglob('*')):
    if path.is_file() and path.name != 'controller.lock':
        h = hashlib.sha256()
        with path.open('rb') as f:
            for chunk in iter(lambda: f.read(1048576), b''):
                h.update(chunk)
        manifest[str(path)] = dict(bytes=path.stat().st_size, sha256=h.hexdigest())
with (OUT / 'artifact_manifest.json').open('x') as f:
    json.dump(manifest, f, indent=2)
print('Archived evidence;', len(manifest), 'files hashed')
