from pathlib import Path
import ast,csv,hashlib,json,subprocess
ROOT=Path('/home/yeranis/results/gnssfgo_eth1_covariance_quality_20261008')
BASE=Path('/home/yeranis/results/gnssfgo_eth1_wnoa_mechanism_20261007')
def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        for b in iter(lambda:f.read(2**20),b''):h.update(b)
    return h.hexdigest()
checks={}
for name in ['summary.json','selected_records.json','inputs_before.json','inputs_after.json',
             'presentation/late_revisions.json','presentation/uncertainty_ranking.png','presentation/revision_evolution.png']:
    assert sha(ROOT/name)==sha(ROOT/'repeat'/name),name
    checks[name]=sha(ROOT/name)
protected=json.loads((ROOT/'inputs_after.json').read_text())
for p,h in protected.items():assert sha(p)==h,p
counts={}
for m in ['off','on']:
    byid={};byinterval={}
    with (BASE/'inputs'/f'{m}_events.csv').open() as f:
        for r in csv.reader(f):
            if r[0]!='B':continue
            key=(r[2],r[3],r[6],r[7]);interval=(r[2],r[8],r[9])
            if key in byid:assert byid[key][0]==interval
            if interval in byinterval:assert byinterval[interval]==key
            byid[key]=(interval,r[5],r[1]);byinterval[interval]=key
    n=0
    with (BASE/f'{m}_wnoa/snapshot_final/ledger.csv').open() as f:
        for r in csv.DictReader(f):
            key=(r['source'],r['session'],r['from_index'],r['to_index'])
            expected=((r['source'],r['from_ns'],r['to_ns']),r['revision'],r['arrival_ns'])
            assert byid[key]==expected,(m,key)
            n+=1
    assert n==1770;counts[m]=n
for p in Path(__file__).parent.glob('*.py'):ast.parse(p.read_text(),str(p))
subprocess.run(['git','-C','/home/yeranis/repos/gnssFGO','diff','--check'],check=True)
result=dict(passed=True,byte_identical_repeat_outputs=checks,protected_hashes_unchanged=len(protected),
            stable_id_latest_revision_matches_final_graph=counts,script_syntax_pass=True,git_diff_check_pass=True)
with (ROOT/'final_validation.json').open('x') as f:json.dump(result,f,indent=2)
print(json.dumps(result,indent=2))
