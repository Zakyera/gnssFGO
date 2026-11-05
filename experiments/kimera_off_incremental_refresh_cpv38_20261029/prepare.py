"""Prepare new private copies; never edit source/baseline trees."""
from pathlib import Path
import hashlib
import json
import shutil
import subprocess

H = Path(__file__).resolve().parent
W = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
R = Path('/home/yeranis/repos/gnssFGO')
K = W/'runs/kimera_incremental_prior_cpk9_20261019'
P = W/'runs'/H.name
assert not P.exists()
assert shutil.disk_usage(W).free > 8 * 1024**3
def sha(p):
    with Path(p).open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()
def write(p, obj):
    with p.open('x') as f: json.dump(obj, f, indent=2)
old = R/'experiments/eig2_native_refresh_cpv37_20261029'
pins = json.loads((old/'protected_before.json').read_text())
assert all(sha(p) == s for p,s in pins.items())
for root in (old, W/'runs/eig2_native_refresh_cpv37_20261029', K):
    for p in root.rglob('*'):
        if p.is_file(): pins[str(p)] = sha(p)
if (H/'protected_before.json').exists():
    assert json.loads((H/'protected_before.json').read_text()) == pins
else:
    write(H/'protected_before.json', pins)
write(H/'git_before.json', {str(x): subprocess.check_output(
    ['git','-C',str(x),'status','--short','--untracked-files=no'], text=True)
    for x in (R,W/'src/cbsms',W/'src/cbs',W/'src/Kimera-VIO')})
write(H/'container_before.json', subprocess.check_output(['docker','inspect','-f',
    '{{.Name}} {{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'],text=True))
P.mkdir()
shutil.copytree(K/'include',P/'include')
for name in ('NativePriorRefresh.h','OriginalChartPrior.h','CurrentChartFallback.h',
             'RetirementAudit.h','LinearParityAudit.h','VioBackend.cpp'):
    shutil.copy2(K/name,P/name)
for name in ('OffRefresh.h','BpsamRefresh.inc','test.cpp','baseline.cpp','build.py'):
    shutil.copy2(H/name,P/name)
(P/'private_build').mkdir()
shutil.copy2(K/'private_build/libcbs.so',P/'private_build/libcbs.so')
def replace(path, before, after):
    text = path.read_text()
    assert text.count(before) == 1, (path,before)
    path.write_text(text.replace(before,after))
replace(P/'include/cbs/bpsam/bpsam.h', '#include "NativePriorRefresh.h"',
        '#include "NativePriorRefresh.h"\n#include "OffRefresh.h"')
replace(P/'include/cbs/bpsam/bpsam.h', '  ARIA_POINTER_TYPEDEFS(BPSAM);',
        '  ARIA_POINTER_TYPEDEFS(BPSAM);\n#include "BpsamRefresh.inc"')
p = P/'include/cbs/bpsam/incremental_fixed_lag_bpsam_smoother.h'
replace(p, '    const bool cpk9_enabled=cpk9::enabled();',
        '    const bool cpv38_enabled = cpv38::enabled();\n'
        '    if (cpv38_enabled) isam_.cpv38ValidateOff();\n'
        '    const bool cpk9_enabled=cpk9::enabled();')
replace(p, 'if (fl4q::enabled() || (cpk9_enabled',
        'if (cpv38_enabled || fl4q::enabled() || (cpk9_enabled')
replace(p, '    if (fl4q::enabled()) fl4q::accept',
        '    // One extra native step; retain original result slot bookkeeping.\n'
        '    if (cpv38::enabled()) isam_.cpv38Refresh(constrainedKeys);\n\n'
        '    if (fl4q::enabled()) fl4q::accept')
replace(p, '    result.iterations = 1;', '    result.iterations = cpv38::enabled() ? 2 : 1;')
# Only command paths/outputs change; reused object/library inputs stay pinned.
container_k = str(K).replace(str(W),'/workspace/cbs_gtsam4.3')
container_p = str(P).replace(str(W),'/workspace/cbs_gtsam4.3')
for name in ('actual_test_commands.json','build_commands.json'):
    data = (K/name).read_text().replace(container_k,container_p)
    with (P/name).open('x') as f: f.write(data)
write(H/'implementation_before.json', {str(p):sha(p) for root in (H,P)
    for p in root.rglob('*') if p.is_file() and p.name not in
    ('protected_before.json','implementation_before.json')})
print(json.dumps({'prepared':str(P),'protected_files':len(pins)}))
