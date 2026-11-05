"""Fresh pre-capture harness after identified executable-bit copying failure."""
from pathlib import Path
import hashlib,json,shutil
H=Path(__file__).resolve().parent
OLD=H.parent/'leica2_rotation_weight_cpv6_20261024'
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
P=W/'runs/gnssfgo_leica2_rotation_weight_cpv6_20261024/prepared'
assert json.loads((OLD/'closeout.json').read_text())['passed']
assert json.loads((OLD/'status.json').read_text())['state']=='failed'
assert not (P/'full_on_source').exists() and not (P/'full_on_receiver').exists()
assert 'test\', \'-x\'' in (P/'full_on_control/failed.txt').read_text()
for name in ('validate_weight.py','launch.py','prepare_harness.py'):
 s=(OLD/name).read_text()
 s=s.replace('gnssfgo_leica2_rotation_weight_cpv6_20261024','gnssfgo_leica2_rotation_weight_cpv6a_20261024')
 if name=='prepare_harness.py':
  a=" with dest.open('x') as f:f.write(s)\n"
  assert s.count(a)==1
  s=s.replace(a,a+" shutil.copymode(src,dest)\n")
  a="shutil.copy2(B/'input_audit.json',P/'input_audit.json')"
  s=s.replace(a,"assert (P/'receiver_cpk19.sh').stat().st_mode & 0o111\n"+a)
 with (H/name).open('x') as f:f.write(s)
protocol=(OLD/'PROTOCOL.txt').read_text()
protocol+='\nPRE-CAPTURE REPAIR A: first launch stopped before source/receiver creation\nsolely because generated receiver_cpk19.sh lacked execute bits. Old evidence\nand successful cleanup preserved; only file-mode-preserving copy fixed.\nThis is the same single candidate capture, not an optimizer/source retry.\nFresh suffix cpv6a; all model/input/settings/code binaries unchanged.\n'
with (H/'PROTOCOL.txt').open('x') as f:f.write(protocol)
with (H/'REPORT.txt').open('x') as f:f.write('CP-V6A: permission-copy launch repair; no source capture previously started.\nOriginal evidence: '+str(OLD)+'\nSame candidate/library/tests/protocol; no tuning retry.\n')
with (H/'prelaunch_repair.json').open('x') as f:json.dump(dict(original=str(OLD),source_started=False,receiver_started=False,repair='preserve original execute bits in generated files',original_closeout=json.loads((OLD/'closeout.json').read_text())),f,indent=2)
