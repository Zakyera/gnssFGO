"""Freeze completed diagnostic report after final verification; no processes launched."""
import ast,datetime,hashlib,json,shutil
from pathlib import Path
H=Path(__file__).resolve().parent;W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
K=W/'runs'/H.name;P=K/'prepared'
assert not (H/'complete.json').exists()
v=json.loads((H/'verification.json').read_text());assert v['passed']
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
protected=json.loads((P/'protected_manifest.json').read_text())
for p,want in protected.items():assert sha(p)==want,p
for n in ('analysis.json','factor_audit.json'):
 d=json.loads((H/n).read_text());pins=d.get('analysis_input_sha256',d.get('input_sha256'))
 for p,want in pins.items():assert sha(p)==want,p
for p in H.glob('*.py'):ast.parse(p.read_text())
assert 'COMPLETE DIAGNOSTIC; NO FIX' in (H/'REPORT.txt').read_text()
shutil.copy2(H/'status.json',H/'controller_final_status.json')
now=datetime.datetime.now().astimezone().isoformat()
(H/'status.json').write_text(json.dumps(dict(state='complete_diagnostic_root_cause_open',updated=now,
 source_pairs=1,source_retries=0,estimator_fix=False,final_evidence='complete.json'),indent=2)+'\n')
files=[p for p in H.rglob('*') if p.is_file() and '__pycache__' not in p.parts]
files += [p for mode in ('off','on') for p in (P/f'full_{mode}_source/checkpoints').rglob('*') if p.is_file()]
for folder in ('inspection','factor_inspection','factor_inspection_v2'):
 files += [p for p in (K/folder).rglob('*') if p.is_file() and p.suffix not in ('.o',) and p.name not in ('Inspect','Factors')]
out=dict(passed=True,completed=now,diagnostic_complete=True,root_cause_proved=False,fix_promoted=False,
 source_pairs=1,source_retries=0,covariance_analysis=False,nonlinear_stationarity_certified=False,
 scope='10s matched Hesai EIG-2;120native snapshots; native insertion/phase/equation audit',
 verification='verification.json',report='REPORT.txt',output_sha256={str(p):sha(p) for p in files})
with (H/'complete.json').open('x') as f:json.dump(out,f,indent=2)
print(json.dumps({k:v for k,v in out.items() if k!='output_sha256'},indent=2));print('output pins',len(files))
