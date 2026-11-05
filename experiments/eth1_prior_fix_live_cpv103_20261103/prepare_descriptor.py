"""Restore the descriptor required by unchanged two-mode adapter fixture."""
import hashlib,json
from paths import H,P,BASE_H,BASE_P
src=BASE_P/'full_off.json';dst=P/'full_off.json'
assert not dst.exists() and not (H/'prepared.json').exists() and not (H/'start.json').exists()
old=src.read_text()
new=old.replace(BASE_H.name,H.name).replace('11611','11621').replace('11612','11622')
new=new.replace('CP-V99','CP-V103').replace('eth1_cpv99_','eth1_cpv103_')
with dst.open('x') as f:f.write(new)
dst.chmod(src.stat().st_mode&0o777)
assert "choices=['on']" in (P/'run_case_on.py').read_text()
with (H/'descriptor_repair.json').open('x') as f:
    json.dump(dict(passed=True,source=str(src),destination=str(dst),
        original_sha256=hashlib.sha256(old.encode()).hexdigest(),
        copied_sha256=hashlib.sha256(new.encode()).hexdigest(),
        purpose='configuration-only adapter fixture; OFF playback disabled',source_launches=0),f,indent=2)
