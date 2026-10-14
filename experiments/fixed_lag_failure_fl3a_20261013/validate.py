"""Read-only archive/payload equivalence and observer results, not a new replay."""
import csv,json
from pathlib import Path
from driver import ROOT,PREV,sha,protect
def main():
    executed=json.loads((ROOT/'observe.json').read_text())
    assert executed['returncode']==0 and executed['protected_unchanged']
    old=PREV/'full_timed_outputs';new=ROOT/'archive'
    oldfiles={str(p.relative_to(old)) for p in old.rglob('*') if p.is_file()}
    newfiles={str(p.relative_to(new)) for p in new.rglob('*') if p.is_file()}
    assert oldfiles==newfiles,('archive file-set mismatch',oldfiles^newfiles)
    for name in sorted(oldfiles):assert sha(old/name)==sha(new/name),('archive bytes changed',name)
    assert not list(new.glob('.stage_*'))
    assert max(int(p.name) for p in (new/'commits').iterdir())==84956
    diagnostic=ROOT/'diagnostics'
    result={}
    for line in (diagnostic/'result.txt').read_text().splitlines():
        k,v=line.split(maxsplit=1);result[k]=v
    result['trials']=list(csv.DictReader((diagnostic/'trials.csv').open()))
    result['decisions']=list(csv.DictReader((diagnostic/'decisions.csv').open()))
    result['controls']=list(csv.DictReader((diagnostic/'controls.csv').open()))
    result['marginal_chart_check']=list(csv.DictReader((diagnostic/'marginal_chart_check.csv').open()))
    result['archive_files_byte_identical']=len(oldfiles)
    result['protected_files']=len(protect())
    assert json.loads((ROOT/'protected.json').read_text())==protect()
    result['observer_valid']=True
    result['receiver_gate_passed']=False
    result['binary_sha256']=executed['binary_sha256']
    with (ROOT/'validation.json').open('x') as f:json.dump(result,f,indent=2)
    print(json.dumps(result,indent=2))
if __name__=='__main__':main()
