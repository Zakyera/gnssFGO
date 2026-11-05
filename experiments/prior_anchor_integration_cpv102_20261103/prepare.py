"""Fresh byte-identical source copies and immutable predecessor/build input pins."""
import hashlib,json,shutil
from pathlib import Path
H=Path(__file__).resolve().parent;E=H.parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3');K=W/'runs'/H.name
C=W/'runs/persistent_compatibility_cpv76_20261101'
S=C/'backend_fixture1/source_snapshot'
H101=E/'prior_anchor_candidate_cpv101_20261103';K101=W/'runs'/H101.name
def read(p):return json.loads(p.read_text())
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def save(n,v):
    with (H/n).open('x') as f:json.dump(v,f,indent=2)
def host(p):return Path(str(p).replace('/workspace/cbs_gtsam4.3',str(W)))
assert read(H101/'complete.json')['passed']
pins=read(H101/'qualification_v3/inputs.json')
pins.update(read(H101/'complete.json')['output_sha256'])
pins[str(H101/'complete.json')]=sha(H101/'complete.json')
source_pins=read(C/'backend_fixture1/sources.json')
for p in S.rglob('*'):
    if p.is_file():
        expected=source_pins[str(C/p.relative_to(S)).replace(str(W),'/workspace/cbs_gtsam4.3')]
        assert sha(p)==expected,p;pins[str(p)]=expected
for name in ('OriginalChartPrior.h','NativePriorRefresh.h'):
    assert sha(S/name)==sha(C/'release_fixture1/source_snapshot'/name)
commands=read(C/'backend_fixture1/commands.json')
cwd=host(read(W/'runs/kimera_on_incremental_refresh_cpv40_20261029/build_commands.json')['cwd'])
for a in commands['link']:
    if a.endswith(('.o','.a','.so','.so.2.2.1','.so.4.3a1')) and a!=str(C/'VioBackend.cpp.o').replace(str(W),'/workspace/cbs_gtsam4.3'):
        p=host(a) if a.startswith('/workspace/') else cwd/a
        if p.is_file():pins[str(p)]=sha(p)
for p in (C/'private_build/libcbs.so',C/'private_build/libkimera_vio.so',
          C/'backend_fixture1/commands.json',C/'release_fixture1/compatibility_compile_command.json',
          C/'release_fixture1/compatibility_link_command.json',
          W/'runs/kimera_incremental_prior_cpk9_20261019/test.cpp',H/'PROTOCOL.txt',H/'prepare.py'):
    pins[str(p)]=sha(p)
for p,v in pins.items():assert sha(p)==v,p
assert shutil.disk_usage(W).free>10*1024**3
K.mkdir()
shutil.copytree(S,K/'baseline')
shutil.copytree(S,K/'candidate')
for name in ('OriginalChartPrior.h','NativePriorRefresh.h'):
    shutil.copy2(K101/'candidate'/name,K/'candidate'/name)
for name in ('baseline_build','private_build'):
    (K/name).mkdir();shutil.copy2(C/'private_build/libcbs.so',K/name/'libcbs.so')
save('inputs.json',pins)
save('prepared.json',dict(passed=True,root=str(K),baseline=str(S),inputs=len(pins),
                         copied_cbs_sha256=sha(K/'private_build/libcbs.so'),build_cwd=str(cwd),
                         original_kimera_sha256=sha(C/'private_build/libkimera_vio.so')))
print('Prepared fresh CP102 copies; pinned',len(pins),'inputs; no build or source launch.')
