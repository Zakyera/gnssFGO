"""Repair only unlaunched staging JSON; preserve erroneous copies for audit."""
import ast,difflib,hashlib,json,re,shutil
from pathlib import Path
H=Path(__file__).resolve().parent;W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3');OLD=H.parent/'leica2_split_imu_live_pair_cpv112_20261105';P=W/'runs'/H.name/'prepared'
assert not (H/'prepared.json').exists() and not (H/'launch.json').exists()
assert not any(P.glob('full_*_control'))
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def rebase(s):
    s=s.replace(OLD.name,H.name).replace('leica2_cpv112_off_20261104','leica2_cpv112a1_off_20261105').replace('leica2_cpv112_on_20261104','leica2_cpv112a1_on_20261105').replace('LEICA2-CPV112','LEICA2-CPV112-A1')
    return re.sub(r'(?<![\d.])11651(?!\d)','11653',re.sub(r'(?<![\d.])11652(?!\d)','11654',s))
backup=H/'preparation_rejected_json';backup.mkdir();fixed=[]
rows=json.loads((H/'copy_parity.json').read_text())['files'];diff=[]
for row in rows:
    src=Path(row['source']);dst=P/row['path'];assert sha(src)==row['source_sha256']
    if dst.suffix=='.json' and 'input_profiles' not in dst.parts:
        correct=rebase(src.read_text())
        if dst.read_text()!=correct:
            old=backup/row['path'];old.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(dst,old)
            dst.write_text(correct);fixed.append(dict(path=str(dst),rejected_sha256=sha(old),restored_sha256=sha(dst),source_sha256=sha(src)))
    if dst.suffix in ('.py','.sh','.env','.launch','.txt','.json') and 'input_profiles' not in dst.parts:
        diff.extend(difflib.unified_diff(src.read_text().splitlines(True),dst.read_text().splitlines(True),fromfile=str(src),tofile=str(dst)))
for name in ('input_audit.json','expected_headers.json'):
    for rel in (name,'off/'+name,'on/'+name):assert sha(P/rel)==sha(W/'runs'/OLD.name/'prepared'/rel)
assert sha(P/'transport_tools/runtime_frame_fixture.json')==sha(W/'runs'/OLD.name/'prepared/transport_tools/runtime_frame_fixture.json')
with (H/'preparation_repair.json').open('x') as f:json.dump(dict(passed=True,repaired=fixed,source_runs=0,receiver_runs=0),f,indent=2)
with (H/'repaired_copy.diff').open('x') as f:f.write(''.join(diff))
s=(H/'preflight_tests.py').read_text().replace("save('validator_test.json'","save('validator_test_v2.json'")
ast.parse(s)
with (H/'preflight_tests_v2.py').open('x') as f:f.write(s)
print(json.dumps(dict(restored=len(fixed),no_captures=True)))
