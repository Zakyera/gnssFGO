"""Retained artifact identities and exact private inheritance after capture."""
from pathlib import Path
import json
from prepare import HERE,BUILD,WS,sha
counts={}
for relative in ['shared_linearization_s3_capture_20261012/evidence/capture/protected_manifest.json',
                 'shared_linearization_s4_timing_20261012/hashes.json',
                 'shared_linearization_s4_eigenvalues_20261012/hashes.json',
                 'shared_linearization_s4_certificate_20261012/hashes.json']:
    manifest=json.loads((HERE.parent/relative).read_text())
    for p,h in manifest.items():assert sha(Path(p))==h,p
    counts[relative]=len(manifest)
for name in ['NativeGaussianHistory.h','ShadowObserver.h','Recording.h','test_native_gaussian.cpp','control.cpp']:
    assert (BUILD/'source'/name).read_bytes()==(WS/'runs/shared_linearization_s3_capture_20261012/release_1/source'/name).read_bytes(),name
assert (BUILD/'source/NativeGaussianHistory.cpp').read_bytes()==(WS/'runs/shared_linearization_s4_certificate_20261012/driver/candidate.cpp').read_bytes()
for name in ['RankCertificate.h','Timing.h']:
    assert (BUILD/'source'/name).read_bytes()==(WS/'runs/shared_linearization_s4_certificate_20261012/driver'/name).read_bytes()
for name in ['incremental_fixed_lag_smoother_ext.cpp','incremental_fixed_lag_smoother_ext_with_fallback.cpp']:
    assert (BUILD/'overlay'/name).read_bytes()==(WS/'runs/shared_linearization_s3_capture_20261012/release_1/overlay'/name).read_bytes()
for name in ['Provider.h']:
    assert (HERE/name).read_bytes()==(BUILD/'source'/name).read_bytes()
result=dict(passed=True,prior_manifests=counts,
    gaussian_backend_and_rank_code_unchanged=True,lifecycle_hooks_unchanged=True,
    private_bridge_sha256=sha(BUILD/'libglim_ros.so'),
    private_cpu_sha256=sha(BUILD/'libgtsam_points.so.1.2.1'))
with (HERE/'final_checks.json').open('x') as f:json.dump(result,f,indent=2)
print(json.dumps(result,indent=2))
