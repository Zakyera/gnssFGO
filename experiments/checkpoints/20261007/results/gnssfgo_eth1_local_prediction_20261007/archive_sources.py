"""Freeze the exact numerical helpers/tests alongside the compiled replay."""
from pathlib import Path
import hashlib, json, shutil, subprocess

root = Path(__file__).resolve().parent
repo = Path('/home/yeranis/repos/gnssFGO')
snapshot = root / 'source_snapshot'
assert not snapshot.exists(), 'Never overwrite a retained source snapshot'
paths = [
    'online_fgo/include/offline_process/' + name for name in [
        'FullHistoryPublicationRefinement.h', 'TransactionalISAM2Rebuild.h',
        'StationaryQRConsensus.h', 'StableGaussianCost.h',
        'BoundedISAM2Refinement.h', 'PublicationRoundoffAcceptance.h',
        'SourceWnojModel.h', 'CausalSupportClock.h']]
paths += ['online_fgo/test/' + name for name in [
    'test_stable_gaussian_cost.cpp', 'test_transactional_isam2_rebuild.cpp',
    'test_stationary_qr_consensus.cpp', 'test_bounded_isam2_refinement.cpp']]
hashes = {}
for name in paths:
    dst = snapshot / name
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(repo / name, dst)
    hashes[name] = hashlib.sha256(dst.read_bytes()).hexdigest()
out = dict(head=subprocess.check_output(['git', '-C', str(repo), 'rev-parse', 'HEAD'], text=True).strip(),
           source_sha256=hashes)
(root / 'source_provenance.json').write_text(json.dumps(out, indent=2) + '\n')
print(json.dumps(out, indent=2))
