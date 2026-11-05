"""Create the isolated logging-only CP-K1 overlay once; no baseline edits."""
from pathlib import Path
import ast,difflib,hashlib,json,shutil
H=Path(__file__).resolve().parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
U=W/'runs/fixed_lag_stopping_fl4u_20261017'
R=W/'runs/kimera_failed_factor_cpk1_20261019'
P=W/'runs/gnssfgo_eth1_local_cov_full_cpr7_20261019/prepared'
def cp(p):return str(p).replace('/home/yeranis/repos/V4RL','/workspace')
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def once(s,a,b):
 assert s.count(a)==1,(a,s.count(a));return s.replace(a,b)
assert not R.exists()
protected=json.loads((P/'protected_manifest.json').read_text())
protected.update(json.loads((U/'pre_capture_receipt.json').read_text())['diagnostic_manifest'])
for p,h in protected.items():assert sha(p)==h,p
R.mkdir()
for n in ['include','private_build','kimera_profile']:
 shutil.copytree(U/n,R/n,symlinks=True,ignore=shutil.ignore_patterns('libkimera_vio.so'))
for n in ['VioBackend.cpp','OriginalChartPrior.h','RetirementAudit.h','LinearParityAudit.h','test.cpp']:
 shutil.copy2(U/n,R/n)
for n in ['grandtour_eth1_kimera_stereo.launch','kimera_vio_ros_newer_college_2021_stereo.launch','kimera_vio_ros.launch']:
 (R/n).write_text((U/n).read_text().replace(cp(U),cp(R)))
old=(U/'CurrentChartFallback.h').read_text()
s=once(old,'// Called only AFTER the unchanged decision has failed.',(H/'FactorProbe.inc').read_text()+'\n// Called only AFTER the unchanged decision has failed.')
s=once(s,'  out<<"done,1\\n";','  factorProbe(stamp,graph,theta,before);\n  out<<"done,1\\n";')
(R/'CurrentChartFallback.h').write_text(s)
header=R/'include/gtsam_unstable/slam/SmartStereoProjectionPoseFactor.h'
before=header.read_text();after=once(before,'  /// shorthand for base class type',(H/'SmartAccess.inc').read_text()+'\n  /// shorthand for base class type');header.write_text(after)
(H/'instrumentation.diff').write_text(''.join(difflib.unified_diff(old.splitlines(True),s.splitlines(True),fromfile='FL4u',tofile='CP-K1'))+'\n'+''.join(difflib.unified_diff(before.splitlines(True),after.splitlines(True),fromfile='original Smart',tofile='diagnostic accessors')))
for n in ['build_commands.json','actual_test_commands.json']:
 c=json.loads((U/n).read_text())
 for k in ['compile','link']:c[k]=[a.replace(cp(U),cp(R)) for a in c[k]]
 (R/n).write_text(json.dumps(c,indent=2))
for n in ['build.py','sanitize.py','probe_test.cpp']:
 s=(U/n).read_text().replace('FL4U_BUILD_TAG','CPK1_BUILD_TAG')
 if n=='probe_test.cpp':
  s=once(s,'  setenv("FL4U_PROBE",argv[1],1);', '  setenv("FL4U_PROBE",argv[1],1);\n  std::string factor_path=std::string(argv[1])+".factors";setenv("CPK1_PROBE",factor_path.c_str(),1);')
 (R/n).write_text(s)
s=(U/'run_case.py').read_text().replace('11443','11454')
s=once(s,"FL4U_PROBE=str(D/'failed_decision.txt'))", "FL4U_PROBE=str(D/'failed_decision.txt'),CPK1_PROBE=str(D/'factor_derivatives.txt'))")
s=once(s,"            assert 'done,1' in (D/'failed_decision.txt').read_text()", "            assert 'done,1' in (D/'failed_decision.txt').read_text()\n            assert 'done,1' in (D/'factor_derivatives.txt').read_text()")
s=once(s,"for bad in ['FL4U','FL4Q',", "for bad in ['CPK1','FL4U','FL4Q',")
(R/'run_case.py').write_text(s)
shutil.copy2(H/'PROTOCOL.txt',R/'PROTOCOL.txt')
(R/'protected_before.json').write_text(json.dumps(protected,indent=2))
for p in R.glob('*.py'):ast.parse(p.read_text())
print(json.dumps(dict(prepared=str(R),protected_count=len(protected),profile_unchanged=True)))
