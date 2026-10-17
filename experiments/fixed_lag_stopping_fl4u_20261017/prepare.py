"""Mechanical isolated diagnostic overlay; no production source edits."""
from pathlib import Path
import ast,difflib,hashlib,json,shutil
H=Path(__file__).resolve().parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
Q=W/'runs/fixed_lag_current_chart_fl4q_20261016'
T=W/'runs/gnssfgo_fixed_lag_live120_off_on_fl4t_20261016/prepared'
R=W/'runs/fixed_lag_stopping_fl4u_20261017'
def cp(p):return str(p).replace('/home/yeranis/repos/V4RL','/workspace')
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def once(s,a,b):
    assert s.count(a)==1,(a,s.count(a));return s.replace(a,b)
protected=json.loads((T/'protected_manifest.json').read_text())
for p,h in protected.items():assert sha(p)==h,p
for p in (H.parent/'fixed_lag_live_fl4t_20261016').rglob('*'):
    if p.is_file():protected[str(p)]=sha(p)
for p in (T/'live120_off_source').glob('*'):
    if p.is_file():protected[str(p)]=sha(p)
assert not R.exists();R.mkdir()
for n in ['include','private_build']:
    shutil.copytree(Q/n,R/n,symlinks=True,ignore=shutil.ignore_patterns('libkimera_vio.so'))
for n in ['VioBackend.cpp','OriginalChartPrior.h','RetirementAudit.h','LinearParityAudit.h','test.cpp']:
    shutil.copy2(Q/n,R/n)
shutil.copytree(T/'live120_off_source/kimera_profile',R/'kimera_profile')
for n in ['grandtour_eth1_kimera_stereo.launch','kimera_vio_ros_newer_college_2021_stereo.launch','kimera_vio_ros.launch']:
    s=(T/n).read_text().replace(cp(T),cp(R));(R/n).write_text(s)
original=(Q/'CurrentChartFallback.h').read_text();s=original
s=once(s,'#include <chrono>','#include <chrono>\n#include <array>\n#include <Eigen/Eigenvalues>')
s=once(s,'inline Decision select(', (H/'FailureProbe.inc').read_text()+'\ninline Decision select(')
s=once(s,'size_t limit=10) {','size_t limit=10,double audit_stamp=-1) {')
s=once(s,'    double previous=r.before;','    double previous=r.before;\n    std::vector<std::array<double,4>> trials;')
s=once(s,'      const double e=cost(graph,v,theta);','      const double e=cost(graph,v,theta);\n      trials.push_back({double(r.iterations),e,lm.error(),lm.lambda()});')
s=once(s,'    if(!(r.after<r.before-tolerance))\n      throw',
    '    if(!(r.after<r.before-tolerance)) {\n      stoppingProbe(audit_stamp,graph,theta,before,full,lm.values(),r,trials);\n      throw')
s=once(s,'"FL4Q no decreasing current-chart LM step");','"FL4Q no decreasing current-chart LM step");\n    }')
s=once(s,'select(graph,theta,initialized,isam.calculateEstimate());','select(graph,theta,initialized,isam.calculateEstimate(),10,stamp);')
(R/'CurrentChartFallback.h').write_text(s)
(H/'instrumentation.diff').write_text(''.join(difflib.unified_diff(original.splitlines(True),s.splitlines(True),fromfile='FL4q',tofile='FL4u logging only')))
for n in ['build_commands.json','actual_test_commands.json']:
    c=json.loads((Q/n).read_text())
    for k in ['compile','link']:c[k]=[a.replace(cp(Q),cp(R)) for a in c[k]]
    (R/n).write_text(json.dumps(c,indent=2))
for n in ['probe_test.cpp','PROTOCOL.txt','run_case.py','build.py','sanitize.py']:
    shutil.copy2(H/n,R/n)
for n in ['test_passed.json','sanitizer_passed.json']:assert not (R/n).exists()
(R/'protected_before.json').write_text(json.dumps(protected,indent=2))
for p in R.glob('*.py'):ast.parse(p.read_text())
print(R)
