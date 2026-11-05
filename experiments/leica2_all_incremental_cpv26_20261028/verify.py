"""Read-back closeout; never runs or changes an estimator."""
import hashlib
import importlib.util
import json
import subprocess
import sys
from pathlib import Path
sys.dont_write_bytecode = True
import numpy as np
H = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('cpv26', H/'analyze.py')
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)
assert not (H/'closeout.json').exists()
pins = json.loads((H/'inputs.json').read_text())
for path, digest in pins.items():
    assert m.sha(path) == digest, path
products = np.load(H/'publication_products.npz')
checks = {}
for mode in ('off', 'on'):
    t,p,r = m.a.load_curve(H/f'{mode}_glim_chained.tum')
    expected = products[mode+'_chained_body_T']
    assert np.array_equal(t, products[mode+'_timestamp_ns'])
    delta = max(float(np.max(np.abs(p-expected[:,:3,3]))),
                float(np.max(np.abs(r-expected[:,:3,:3]))))
    assert delta < 1e-10
    checks[mode] = dict(rows=len(t), integer_timestamp_parity=True, pose_roundtrip_max=delta)
checkpoint = subprocess.run(['python3', str(H.parent/'checkpoints/20261023_hesai_four_sequences/verify_checkpoint.py'), '--originals'],
                            capture_output=True, text=True, check=True).stdout.strip()
assert 'PASS: 4 cases, 667 saved files; originals=True' in checkpoint
output_hashes = {str(p):m.sha(p) for p in H.iterdir() if p.is_file()}
with (H/'closeout.json').open('x') as f:
    json.dump(dict(pass_=True, input_pins_verified=len(pins), product_readback=checks,
                   hesai_checkpoint=checkpoint, new_estimator_or_container_jobs_launched=0,
                   source_or_model_changes=False, production_promotion=False,
                   output_sha256=output_hashes), f, indent=2, allow_nan=False)
print(json.dumps(dict(pass_=True, input_pins_verified=len(pins), product_readback=checks,
                     hesai_checkpoint=checkpoint), indent=2))
