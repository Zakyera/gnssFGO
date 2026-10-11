"""Additional E2/runtime preservation gates; no estimator execution."""
import ast
import collections
import csv
import hashlib
import json
from pathlib import Path
import subprocess

HERE=Path(__file__).resolve().parent
RUN=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_e1_e2_live60_20261011')
ROOT=RUN/'prepared'
LIVE=ROOT/'live60_on_receiver'
def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        for b in iter(lambda:f.read(1048576),b''):h.update(b)
    return h.hexdigest()
def rows(p):
    with p.open() as f:return list(csv.DictReader(f))
v=json.loads((ROOT/'live60_on_validation.json').read_text());assert v['passed']
manifest=json.loads((ROOT/'protected_manifest.json').read_text())
for p,h in manifest.items():assert sha(p)==h,p
s=v['summary']
assert s['active_factor_count']==s['wnoa_factor_count']+s['relative_factor_count']+3
last={r['kind']:r for r in rows(LIVE/'receiver/e2_memo.csv')}
for r in last.values():assert int(r['hits'])>0 and int(r['failures'])==0
arrivals=collections.Counter();ordinals={};end=0
with (LIVE/'events.csv').open() as f:
    for r in csv.reader(f):
        if not r:continue
        if r[0]=='E':end+=1
        if r[0]!='B':continue
        key=(r[2],r[3],r[6],r[7],r[5]);arrivals[key]+=1
        ordinals[(r[2],r[3],r[4],r[5])]=key
accounted=collections.Counter()
for r in rows(LIVE/'receiver/causal_dispatch.csv'):
    accounted[tuple(r[k] for k in ['source','session','from_index','to_index','revision'])]+=1
for r in rows(LIVE/'receiver/unsupported_beliefs.csv'):
    accounted[ordinals[(r['source'],r['session'],r['ordinal'],r['revision'])]]+=1
assert end==1 and arrivals==accounted and all(n==1 for n in arrivals.values())
samples=[json.loads(l) for l in (RUN/'resource_samples.jsonl').open()]
assert len(samples)>=2
steps=rows(LIVE/'receiver/publication_steps.csv')
repos={}
for name in ['gnssFGO','V4RL/cbs_gtsam4.3/src/glim_ros1','V4RL/cbs_gtsam4.3/src/cbs','V4RL/cbs_gtsam4.3/src/gtsam_points','V4RL/cbs_gtsam4.3/src/Kimera-VIO','V4RL/cbs_gtsam4.3/src/cbsms']:
    p=Path('/home/yeranis/repos')/name
    repos[name]=dict(head=subprocess.check_output(['git','-C',str(p),'rev-parse','HEAD'],text=True).strip(),
                    status=subprocess.check_output(['git','-C',str(p),'status','--short'],text=True))
    if name!='gnssFGO':assert not repos[name]['status']
products={p.name:sha(p) for p in (LIVE/'receiver').iterdir() if p.is_file()}
for p in [*HERE.glob('*.py'),*ROOT.glob('*.py')]:ast.parse(p.read_text())
result=dict(passed=True,protected_hashes=len(manifest),cache_last=last,source_events_exactly_once=len(arrivals),explicit_end_markers=end,
            publications=s['support_states'],factor_count=s['active_factor_count'],wnoa=s['wnoa_factor_count'],relative=s['relative_factor_count'],
            publication_steps=len(steps),recovery_steps=sum(r['recovery']=='1' for r in steps),
            recovery_publications=len({r['index'] for r in steps if r['recovery']=='1'}),
            input_journal_sha256=sha(LIVE/'events.csv'),output_hashes=products,resource_snapshots=len(samples),repositories=repos)
with (RUN/'final_checks.json').open('x') as f:json.dump(result,f,indent=2)
print(json.dumps({k:v for k,v in result.items() if k not in ['output_hashes','repositories']},indent=2))
