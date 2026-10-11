from experiment import ROOT, file_sha
import json
assert not (ROOT/'pilot_on_control').exists()
assert json.loads((ROOT/'repeatability/validation.json').read_text())['passed']
names=['repeatability_protocol.txt','numerical_equivalence.py','check_repeatability.py',
       'validate_numerical_case.py','evaluate_full40.py','freeze_acceptance.py']
manifest={str(ROOT/n):file_sha(ROOT/n) for n in names}
with (ROOT/'acceptance_manifest.json').open('x') as f: json.dump(manifest,f,indent=2)
print('Numerical acceptance and evaluation frozen before 40 s launch.')
