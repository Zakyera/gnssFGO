"""Mechanical fresh-path copy of CP95; never copies capture outputs or runs sources."""
import json, shutil
from pathlib import Path
H=Path(__file__).resolve().parent
E=H.parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
old='eth1_split_imu_live_pair_cpv95_20261103_attempt3'
B=E/'eth1_split_imu_live_pair_cpv95_20261103_continuation1'
A=E/old
Q=W/'runs'/old/'prepared'
P=W/'runs'/H.name/'prepared'
assert json.loads((B/'complete.json').read_text())['passed']
assert json.loads((B/'post_closeout.json').read_text())['passed']
assert not P.exists()
def copy(src,dst):
    dst.parent.mkdir(parents=True,exist_ok=True)
    shutil.copy2(src,dst)
    if src.suffix in ('.py','.sh','.env','.json','.launch','.txt'):
        text=dst.read_text().replace(old,H.name)
        if src.name in ('source.env','run_case_on.py','full_off.json','full_on.json'):
            text=text.replace('11601','11611').replace('11602','11612')
        dst.write_text(text)
for src in Q.rglob('*'):
    rel=src.relative_to(Q)
    if any(part.startswith('full_') and part.endswith(('_control','_source','_receiver')) for part in rel.parts):continue
    if '__pycache__' in rel.parts:continue
    if src.is_file() and src.name not in ('controller.lock','protected_manifest.json','preflight.json','validate_source.py','validate_local.py','validate_case.py'):
        copy(src,P/rel)
for name in ('evaluate.py','verify_report.py','validate_source.py','validate_local.py','validate_case.py',
             'frame_off.py','frame_on.py','test_validator.py','injection_schema.py','report_preflight.py'):
    shutil.copy2(B/name,H/name)
text=(H/'evaluate.py').read_text().replace('CP-V95','CP-V99')
(H/'evaluate.py').write_text(text)
text=(H/'report_preflight.py').read_text()
text=text.replace('from paths import H,E,W,P','from paths import H,E,W,P,BASE_H,BASE_P')
text=text.replace("assert json.loads((H/'off_validated.json').read_text())['passed']", "assert json.loads((BASE_H/'complete.json').read_text())['passed']")
text=text.replace("P/'full_off_source'","BASE_P/'full_off_source'").replace("P/'full_off_receiver", "BASE_P/'full_off_receiver")
text=text.replace('new_off','saved_original_off')
(H/'report_preflight.py').write_text(text)
text=(A/'controller.py').read_text()
text=text.replace("H=Path(__file__).resolve().parent;E=H.parent;W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')\nP=W/'runs'/H.name/'prepared';V=Path('/home/yeranis/results')/H.name\nPY='/home/yeranis/.local/share/pipx/venvs/evo/bin/python'",'from paths import H,E,W,P,V,PY')
text=text.replace('ETH1-CPV95','ETH1-CPV99')
text=text.replace("assert not (H/'start.json').exists() and read(H/'prepared.json')['passed']", "assert not (H/'start.json').exists() and read(H/'prepared.json')['passed']\nassert read(H/'report_preflight.json')['passed'] and read(H/'validator_test.json')['passed']")
text=text.replace("str(P/script)","str(H/script)")
text=text.replace("P/f'full_{mode}_source/candidate_validation.json',P/f'full_{mode}_source/local_provenance.json'", "H/f'{mode}_candidate_validation.json',H/f'{mode}_local_provenance.json'")
text=text.replace("    assert read(H/'report_verification.json')['passed'];count=hashes()", "    assert read(H/'report_verification.json')['passed']\n    step('compare_repeat',[PY,'-B',str(H/'compare_repeat.py')],timeout=1200)\n    assert read(H/'repeat_comparison.json')['passed'];count=hashes()")
(H/'controller.py').write_text(text)
text=(B/'launch.py').read_text().replace('ETH1-CPV95-CONTINUATION1','ETH1-CPV99')
text=text.replace('User-authorized remaining ETH1 ON with fusion; saved OFF preserved','User-authorized identical-settings ETH1 repeat OFF/ON with fusion')
text=text.replace('ON_only_continuation=True,OFF_rerun=False','fresh_repeat_pair=True,source_captures_authorized=2')
(H/'launch.py').write_text(text)
print(P)
