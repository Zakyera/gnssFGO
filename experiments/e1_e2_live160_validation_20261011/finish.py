"""Exact conservation, preservation and repeat evidence. No experiment launch."""
import ast
import collections
import csv
import hashlib
import json
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

HERE=Path(__file__).resolve().parent;REPO=HERE.parents[1]
RUN=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_e1_e2_live160_20261011')
OUT=RUN.parent/'gnssfgo_e1_e2_live160_validation_20261011';ROOT=RUN/'prepared'
LIVE=ROOT/'live160_on_receiver';A=LIVE/'receiver';B=OUT/'live160_on_numerical_repeat'
def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        for b in iter(lambda:f.read(1048576),b''):h.update(b)
    return h.hexdigest()
def rows(p):
    with p.open() as f:return list(csv.DictReader(f))
v=json.loads((OUT/'live160_on_validation.json').read_text());assert v['passed']
manifest=json.loads((ROOT/'protected_manifest.json').read_text())
for p,h in manifest.items():assert sha(p)==h,p
old_evidence=json.loads((RUN/'capture_evidence.json').read_text())
for p,h in old_evidence['source_log_hashes'].items():assert sha(p)==h,p
assert sha(LIVE/'events.csv')==old_evidence['journal_sha256']
old_archive=REPO/'experiments/e1_e2_live160_20261011'
copied=json.loads((old_archive/'ARCHIVE_MANIFEST.json').read_text())['copied']
for rel,item in copied.items():
    assert sha(old_archive/rel)==sha(item['original'])==item['sha256'],rel
s=v['summary'];assert s['active_factor_count']==s['wnoa_factor_count']+s['relative_factor_count']+3
arrivals=collections.Counter();ordinals={};end=0
with (LIVE/'events.csv').open() as f:
    for r in csv.reader(f):
        if not r:continue
        if r[0]=='E':end+=1
        if r[0]!='B':continue
        key=(r[2],r[3],r[6],r[7],r[5]);arrivals[key]+=1
        ordinals[(r[2],r[3],r[4],r[5])]=key
accounted=collections.Counter()
for r in rows(A/'causal_dispatch.csv'):
    accounted[tuple(r[k] for k in ['source','session','from_index','to_index','revision'])]+=1
for r in rows(A/'unsupported_beliefs.csv'):
    accounted[ordinals[(r['source'],r['session'],r['ordinal'],r['revision'])]]+=1
assert end==1 and arrivals==accounted and all(n==1 for n in arrivals.values())
def payload(p):
    omit={'publication_convergence.csv':{'seconds'},'revision_events.csv':{'update_seconds'},'stream_progress.csv':{'wall_unix_ns'}}
    if p.name in omit:return [{k:v for k,v in r.items() if k not in omit[p.name]} for r in rows(p)]
    if p.name=='summary.json':return {k:v for k,v in json.loads(p.read_text()).items() if k not in ['wall_seconds','revision_update_seconds']}
    return sha(p)
assert {p.name for p in A.iterdir() if p.is_file()}=={p.name for p in B.iterdir() if p.is_file()}
exact={p.name:payload(p)==payload(B/p.name) for p in A.iterdir() if p.is_file()}
repos={}
for name in ['glim_ros1','cbs','gtsam_points','Kimera-VIO','cbsms']:
    p=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/src')/name
    key='V4RL/cbs_gtsam4.3/src/'+name
    head=subprocess.check_output(['git','-C',str(p),'rev-parse','HEAD'],text=True).strip()
    status=subprocess.check_output(['git','-C',str(p),'status','--short'],text=True)
    assert not status and head==old_evidence['repositories'][key]['head']
    repos[name]=dict(head=head,status=status)
tests={}
for name in ['receiver_release','receiver_sanitize']:
    r=ET.parse(OUT/(name+'.xml')).getroot();assert r.attrib['failures']=='0' and r.attrib['errors']=='0'
    tests[name]=r.attrib
for p in HERE.glob('*.py'):ast.parse(p.read_text())
result=dict(passed=True,protected_original_hashes=len(manifest),old_archive_files_unchanged=len(copied),
    source_logs_unchanged=len(old_evidence['source_log_hashes']),source_versions_exactly_once=len(arrivals),
    explicit_end_markers=end,all_non_timing_products_exact=all(exact.values()),repeat_products=exact,
    numerical_repeat=v['numerical_repeat'],source_repositories=repos,focused_tests=tests,
    repeat_wall_seconds=json.loads((B/'summary.json').read_text())['wall_seconds'],
    repeat_files_sha256={p.name:sha(p) for p in B.iterdir() if p.is_file()},
    limits='Focused existing test executables rerun; no dependency/device instrumentation, no new full build/suite or source capture. Replay wall time is not a controlled performance comparison.')
with (OUT/'final_checks.json').open('x') as f:json.dump(result,f,indent=2)
print(json.dumps({k:v for k,v in result.items() if k not in ['repeat_files_sha256','source_repositories']},indent=2))
