"""Validate the observed blocked gate, not certify the incomplete FL2 receiver."""
import json,hashlib,sys
from pathlib import Path
from driver import ROOT,HERE,protected
def main():
    a=(ROOT/'gate_release3.log').read_bytes();b=(ROOT/'gate_release4.log').read_bytes()
    san=(ROOT/'gate_sanitize1.log').read_text()
    assert a==b, 'Release repeat differs'
    for name in ['gate_release3','gate_release4','gate_sanitize1']:
        info=json.loads((ROOT/(name+'.json')).read_text())
        text=(ROOT/(name+'.log')).read_text()
        assert info['returncode']==2 and info['protected_unchanged']
        assert 'CHECKS 85\nGATE BLOCKED\n' in text
        assert 'FAIL ' not in text
        assert 'BLOCKER strict_retained_publication_failed' in text
    assert 'ERROR: AddressSanitizer' not in san and 'runtime error:' not in san
    assert json.loads((ROOT/'protected.json').read_text())==protected()
    result=dict(overall_FL2_gate_passed=False,status='BLOCKED_STRICT_RETAINED_PUBLICATION',
      supporting_checks_per_execution=85,release_byte_identical=True,
      focused_sanitizer_findings=False,protected_files_unchanged=len(protected()),
      explicitly_not_done=['complete receiver lifecycle','pending/interpolated retirement suite',
       'real journal replay','source or live run','speed test','default promotion'],
      files={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [
        *HERE.glob('*.h'),*HERE.glob('*.cpp'),ROOT/'build_release5',ROOT/'build_sanitize2',
        ROOT/'gate_release3.log',ROOT/'gate_release4.log',ROOT/'gate_sanitize1.log']})
    with (ROOT/'validation.json').open('x') as f:json.dump(result,f,indent=2)
    print(json.dumps({k:v for k,v in result.items() if k!='files'},indent=2))
if __name__=='__main__':main()
