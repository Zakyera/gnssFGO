"""Mechanical mission-specific copies of proven read-only input audits."""
import ast,difflib,hashlib,json
from pathlib import Path
H=Path(__file__).resolve().parent;S=H.parent/'eig2_split_imu_live_pair_cpv108_20261104'
assert not (H/'available_data_review.json').exists()
diff=[];records=[]
for name in ('inspect_available.py','audit_inputs.py'):
    src=S/name;old=src.read_text();s=old
    for a,b in [('EIG-2','SPX-3'),('2024-11-03-13-59-54','2024-11-02-17-43-10'),
                ('1730638816563546034','1730565899872888651'),('1730639171583546034','1730566096892888651'),
                ('5700000000','29000000000'),('offset_s=5.7','offset_s=29.0'),
                ('355.02','197.02'),('35502','19702'),('3551','1970'),('3550','1970'),
                (')*1e-9>354',')*1e-9>196')]:s=s.replace(a,b)
    if name=='inspect_available.py':
        s=s.replace("assert not inventory['alphasense_imu']['exists']","assert inventory['alphasense_imu']['exists']")
        s=s.replace("status='blocked_missing_original_alphasense_imu_bag'","status='inventory_present_pending_raw_audit'")
        s=s.replace('Inventory and calibration comparisons only; no Alpha input, full-window or launch qualification','Inventory and calibration comparisons only; raw selected-window audit and launch admission still required')
    ast.parse(s)
    with (H/name).open('x') as f:f.write(s)
    records.append(dict(name=name,source=str(src),source_sha256=hashlib.sha256(old.encode()).hexdigest(),sha256=hashlib.sha256(s.encode()).hexdigest()))
    diff.extend(difflib.unified_diff(old.splitlines(True),s.splitlines(True),fromfile=str(src),tofile=str(H/name)))
with (H/'input_tools_copy.json').open('x') as f:json.dump(records,f,indent=2)
with (H/'input_tools_copy.diff').open('x') as f:f.write(''.join(diff))
print('Staged raw inspection tools only; no sensor or receiver execution')
