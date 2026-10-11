"""Verify immutable references and archive small evidence, not large replays."""
import ast,hashlib,json,re,shutil,statistics,xml.etree.ElementTree as ET
from pathlib import Path
HERE=Path(__file__).resolve().parent
ROOT=Path('/home/yeranis/results/gnssfgo_e1_e2_correctness_audit_20261011')
GPU=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/e1_correctness_audit_20261011')
def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        for b in iter(lambda:f.read(1048576),b''):h.update(b)
    return h.hexdigest()
expected=json.loads((ROOT/'protected.json').read_text())
for p,h in expected.items():assert sha(p)==h,p
assert sha(HERE/'ExactLinearizationMemo.h')==sha(ROOT/'candidate/ExactLinearizationMemo.h')
for p in HERE.glob('*.py'):ast.parse(p.read_text())
actions=['original_unit','candidate_unit','candidate_sanitize','candidate_build','original_micro',
         'candidate_micro','off1','reference_pinned','reference_unpinned','on1','on2','off2']
for action in actions:assert json.loads((ROOT/(action+'.json')).read_text())['passed'],action
for action in ['release','ubsan','cpu_sanitize','micro']:
    assert json.loads((GPU/(action+'.json')).read_text())['passed'],action
for action in ['environment_control',*[s+'_comparison' for s in ['off1','on1','on2','off2']]]:
    result=json.loads((ROOT/(action+'.json')).read_text());assert result.get('passed',result.get('valid'))
tests={}
for p in [*ROOT.glob('*.xml'),*GPU.glob('*.xml')]:
    x=ET.parse(p).getroot();assert int(x.attrib['failures'])==int(x.attrib['errors'])==0
    tests[str(p)]=dict(tests=int(x.attrib['tests']),disabled=int(x.attrib['disabled']),
                      run=sum(t.attrib.get('status')=='run' for t in x.iter('testcase')))
comparisons={case:json.loads((ROOT/(case+'_comparison.json')).read_text()) for case in ['off1','on1','on2','off2']}
micro={}
for kind in ['original','candidate']:
    data={i:[] for i in range(4)}
    for mode,seconds in re.findall(r'MICRO mode=(\d).*?seconds=([0-9.e+-]+)',(ROOT/'logs'/(kind+'_micro.log')).read_text()):
        data[int(mode)].append(float(seconds)*1e6/3000)
    micro[kind]={k:dict(samples=v,median_microseconds=statistics.median(v)) for k,v in data.items()}
matrix={}
for label in ['release','ubsan','cpu_sanitize']:
    values=[float(x) for x in re.findall(r'E1_ERROR .*?max_abs=([0-9.e+-]+)',(GPU/(label+'.log')).read_text())]
    matrix[label]=dict(comparisons=len(values),max_abs=max(values))
preserved=ROOT/'preservation.json'
with preserved.open('x') as f:json.dump(dict(valid=True,protected_files=len(expected),
    candidate={str(p):sha(p) for p in (ROOT/'candidate').iterdir() if p.is_file()},
    e1_executables={str(p):sha(p) for p in GPU.glob('*_test')},
    products={case:{p.name:sha(p) for p in (ROOT/case).iterdir() if p.is_file()} for case in ['off1','reference_pinned','reference_unpinned','on1','on2','off2']}),f,indent=2)
summary=dict(valid=True,tests=tests,e1_matrices=matrix,e2_micro=micro,
    comparisons=comparisons,protected_files=len(expected))
with (ROOT/'audit_summary.json').open('x') as f:json.dump(summary,f,indent=2)
out=HERE/'evidence';out.mkdir(exist_ok=False);manifest=[]
for directory,destination in [(ROOT,out/'cpu'),(GPU,out/'gpu')]:
    destination.mkdir()
    for p in sorted(directory.rglob('*')):
        if not p.is_file() or p.suffix not in ['.json','.xml','.log','.txt']:continue
        if p.parent not in [directory,directory/'logs']:continue
        target=destination/p.relative_to(directory);target.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(p,target)
        manifest.append(dict(source=str(p),archived=str(target.relative_to(HERE)),sha256=sha(p),bytes=p.stat().st_size))
        assert sha(p)==sha(target)
with (HERE/'ARCHIVE_MANIFEST.json').open('x') as f:json.dump(manifest,f,indent=2)
print('Preserved',len(expected),'originals; archived',len(manifest),'evidence files')
print('CPU tests',tests,'E1 matrix diffs',matrix)
print('Replay seconds',{k:v['process_seconds'] for k,v in comparisons.items()})
