"""Independent archive/conservation checks; refuse to promote unrun real replay."""
import csv,hashlib,json
from pathlib import Path
from driver import ROOT,HERE,protected
from export_archive import export

def files(root):
    return {str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest()
            for p in root.rglob('*') if p.is_file()}
def records(root):
    result=[]
    for p in sorted((root/'commits').iterdir(),key=lambda p:int(p.name)):
        with (p/'records.txt').open() as f:result.extend(csv.reader(f))
    return result
def audit(root):
    rows=records(root);active={};absorbed={};counts={}
    for r in rows:
        if r[0]=='belief':
            outcome=r[1];counts[outcome]=counts.get(outcome,0)+1
            identity=tuple(r[2:6]);revision=int(r[6]);start,end,arrival,insertion=map(int,r[8:12])
            assert arrival>=end and insertion>=arrival
            if outcome in ('before_initialization','unsupported_at_end'):continue
            assert identity not in absorbed
            if outcome=='insert':assert identity not in active
            elif outcome=='replace':assert identity in active and revision>active[identity][0]
            elif outcome=='duplicate':assert revision==active[identity][0]
            elif outcome=='stale':assert revision<active[identity][0]
            else:raise AssertionError(outcome)
            if outcome in ('insert','replace'):
                if identity in active:assert (start,end)==active[identity][1:3]
                active[identity]=(revision,start,end,int(r[12]))
                assert len({v[3] for v in active.values()})==len(active),'duplicate active factor slot'
        elif r[0]=='absorbed':
            identity=tuple(r[1:5]);revision,slot=int(r[5]),int(r[6]);assert identity not in absorbed
            old=active.pop(identity);assert old[0]==revision and old[3]==slot;absorbed[identity]=old
        elif r[0]=='publication':assert float(r[3])<=1e-5 and int(r[4])<=20
    marker_count=len([p for p in (root/'ids').rglob('*') if p.is_file()])
    assert marker_count==len(absorbed)
    assert counts.get('insert',0)==len(active)+len(absorbed)
    assert len([r for r in rows if r[0]=='end'])==1
    return dict(events=counts,active_ids=len(active),absorbed_ids=len(absorbed),markers=marker_count)
def main():
    logs={n:(ROOT/(n+'.log')).read_bytes() for n in ('test_release6','test_release7','test_sanitize1')}
    assert logs['test_release6']==logs['test_release7']
    for name,data in logs.items():
        info=json.loads((ROOT/(name+'.json')).read_text());assert info['returncode']==0 and info['protected_unchanged']
        assert data.endswith(b'CHECKS 1004\nFL2B_LIFECYCLE_GATE PASS\n')
        assert b'ERROR: AddressSanitizer' not in data and b'runtime error:' not in data
    assert files(ROOT/'test_release6_outputs')==files(ROOT/'test_release7_outputs')==files(ROOT/'test_release2_outputs')
    audits={}
    for name in ('no_retire','pending','turnover','startup'):
        path=ROOT/'test_release6_outputs'/name;audits[name]=audit(path)
        export(path,ROOT/('export_'+name))
    try:export(ROOT/'test_release6_outputs/pending_failure',ROOT/'must_not_export_incomplete')
    except AssertionError:pass
    else:raise AssertionError('incomplete archive exported')
    for name in ('wire_release5','wire_release6','wire_sanitize1'):
        info=json.loads((ROOT/(name+'.json')).read_text());assert info['returncode']==0 and info['protected_unchanged']
        data=(ROOT/(name+'.log')).read_text();assert 'FL2B_STREAM_COMPLETE' in data
        assert 'ERROR: AddressSanitizer' not in data and 'runtime error:' not in data
        audits[name]=audit(ROOT/(name+'_outputs'))
    assert files(ROOT/'wire_release5_outputs')==files(ROOT/'wire_release6_outputs')==files(ROOT/'wire_release1_outputs')
    export(ROOT/'wire_release5_outputs',ROOT/'export_wire')
    negative=json.loads((ROOT/'wire_truncated2.json').read_text())
    assert negative['returncode']==1 and negative['protected_unchanged']
    assert 'stream disconnected without E marker' in (ROOT/'wire_truncated2.log').read_text()
    assert not any(r[0]=='end' for r in records(ROOT/'wire_truncated2_outputs'))
    regression=json.loads((ROOT/'regression_fl2a.json').read_text());assert regression['returncode']==0
    assert (ROOT/'regression_fl2a.log').read_text().endswith('CHECKS 651\nFL2A_BOUNDARY_GATE PASS\n')
    saved=json.loads((ROOT/'protected.json').read_text());assert saved==protected()
    result=dict(status='FL2B_SYNTHETIC_RECEIVER_PASS',checks=1004,inherited_regression_checks=651,
      release_logs_and_archives_byte_identical=True,focused_sanitizer_findings=False,
      protected_files_unchanged=len(saved),audits=audits,
      not_done=['real journal replay','comparative timing','live capture','default promotion','crash resume'],
      source_hashes={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in HERE.iterdir()
                     if p.is_file() and p.suffix in ('.h','.cpp','.py')},
      binary_hashes={n:hashlib.sha256((ROOT/n).read_bytes()).hexdigest()
                     for n in ('build_release4','build_wire3','build_sanitize1','build_wire_sanitize1')})
    with (ROOT/'validation.json').open('x') as f:json.dump(result,f,indent=2)
    print(json.dumps({k:v for k,v in result.items() if k not in ('audits','source_hashes','binary_hashes')},indent=2))
if __name__=='__main__':main()
