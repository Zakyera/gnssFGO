"""Exclusive private copies and read-only protection of completed evidence."""
import hashlib, json, shutil, subprocess
from pathlib import Path
H = Path(__file__).resolve().parent
W = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
R = H.parents[1]
OLD = W/'runs/kimera_off_incremental_refresh_cpv38_20261029'
P = W/'runs'/H.name
assert not P.exists() and shutil.disk_usage(W).free > 8*1024**3
def sha(p):
    with Path(p).open('rb') as f: return hashlib.file_digest(f,'sha256').hexdigest()
def save(p,v):
    with p.open('x') as f: json.dump(v,f,indent=2)
pins=json.loads((R/'experiments/eig2_off_refresh_full_cpv39_20261029/protected_before.json').read_text())
assert all(sha(p)==digest for p,digest in pins.items())
for root in (OLD, R/'experiments/eig2_off_refresh_full_cpv39_20261029',
             W/'runs/eig2_off_refresh_full_cpv39_20261029',
             Path('/home/yeranis/results/eig2_off_refresh_method1_cpv39_20261029')):
    for p in root.rglob('*'):
        if p.is_file() and p.name != 'open_viewer.log': pins[str(p.resolve())]=sha(p)
save(H/'protected_before.json',pins)
save(H/'git_before.json',{str(p):subprocess.check_output(['git','-C',str(p),'status','--short','--untracked-files=no'],text=True)
    for p in (R,W/'src/cbsms',W/'src/cbs',W/'src/Kimera-VIO')})
save(H/'container_before.json',subprocess.check_output(['docker','inspect','-f','{{.Name}} {{.State.Status}}',
    'cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'],text=True))
P.mkdir();shutil.copytree(OLD/'include',P/'include')
for name in ('NativePriorRefresh.h','OriginalChartPrior.h','CurrentChartFallback.h',
             'RetirementAudit.h','LinearParityAudit.h','VioBackend.cpp','OffRefresh.h','BpsamRefresh.inc'):
    shutil.copy2(OLD/name,P/name)
(P/'private_build').mkdir();shutil.copy2(OLD/'private_build/libcbs.so',P/'private_build/libcbs.so')
old=str(OLD).replace(str(W),'/workspace/cbs_gtsam4.3')
new=str(P).replace(str(W),'/workspace/cbs_gtsam4.3')
for name in ('actual_test_commands.json','build_commands.json'):
    with (P/name).open('x') as f:f.write((OLD/name).read_text().replace(old,new))
test=(OLD/'test.cpp').read_text()
assert test.count('int main() try')==1
with (P/'off_tests.inc').open('x') as f:f.write(test.replace('int main() try','int cpv38_saved_main() try'))
save(H/'prepared.json',dict(private_root=str(P),protected_files=len(pins),free_bytes=shutil.disk_usage(W).free))
print('Prepared private root; protected files',len(pins),flush=True)
