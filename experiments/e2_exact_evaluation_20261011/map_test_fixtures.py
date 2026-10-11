"""Map two historical container mount paths in a private test copy only."""
import hashlib,json
from pathlib import Path
ROOT=Path('/home/yeranis/results/gnssfgo_e2_exact_evaluation_20261011')
VW=Path('/home/yeranis/results/gnssfgo_vw_jacobian_20261011/build')
mapping={'/prior/cached/failure_probe/simultaneous_values.csv':
         '/home/yeranis/results/gnssfgo_publication_cache_20261010/cached/failure_probe/simultaneous_values.csv',
         '/diagnostic/selected_factors.csv':
         '/home/yeranis/results/gnssfgo_derivative_check_20261011/selected_factors.csv'}
s=(VW/'test_twist.cpp').read_text()
for before,after in mapping.items():
    assert s.count(before)==1 and Path(after).is_file();s=s.replace(before,after)
(ROOT/'candidate_build/test_twist.cpp').write_text(s)
with (ROOT/'fixture_mapping.json').open('x') as f:
    json.dump({a:dict(original_mount=b,sha256=hashlib.sha256(Path(a).read_bytes()).hexdigest()) for b,a in mapping.items()},f,indent=2)
print('Only two fixture paths mapped; all original assertions unchanged')
