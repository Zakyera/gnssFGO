"""Inherit exact B0/S3 admission gates with provider-specific test artifacts."""
from pathlib import Path
from prepare import HERE
s=(HERE.parent/'shared_linearization_s3_capture_20261012/preflight.py').read_text()
for line in ["assert json.loads((HERE/'synthetic_oracle.json').read_text())['passed']\n",
             "assert json.loads((HERE/'mixed_oracle.json').read_text())['passed']\n"]:
    assert s.count(line)==1;s=s.replace(line,'')
s=s.replace('11420','11422').replace("HERE/'Recording.h'","HERE/'Provider.h'")
s=s.replace("'asan_1/tests/result.json'","'asan_nopie/tests/result.json'")
s=s.replace('S3_CAPTURE_PREFLIGHT_PASS','S4D_PROVIDER_PREFLIGHT_PASS')
exec(compile(s,__file__,'exec'))
