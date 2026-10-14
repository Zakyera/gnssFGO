"""Verify this bounded adapter gate; never label the whole receiver validated."""
import json,hashlib
from pathlib import Path
from driver import HERE,base,protected
ROOT=base.ROOT
def main():
    logs={n:(ROOT/(n+'.log')).read_bytes() for n in ['test_release3','test_release4','test_sanitize1']}
    assert logs['test_release3']==logs['test_release4'],'Release repeats differ'
    for name,content in logs.items():
        info=json.loads((ROOT/(name+'.json')).read_text());text=content.decode()
        assert info['returncode']==0 and info['protected_unchanged']
        assert text.endswith('CHECKS 651\nFL2A_BOUNDARY_GATE PASS\n')
        assert 'PASS frozen FL2 negative control remains blocked' in text
        assert '\nFAIL ' not in text and 'ERROR: AddressSanitizer' not in text and 'runtime error:' not in text
    saved=json.loads((ROOT/'protected.json').read_text());assert saved==protected()
    metrics={}
    for line in logs['test_release3'].decode().splitlines():
        if line.startswith('METRIC '):
            _,key,value=line.split();metrics.setdefault(key,[]).append(float(value))
    assert metrics['stream_rebuilds']==[74.] and metrics['stream_variables']==[213.]
    assert metrics['stream_slot_high_water']==[144.] and metrics['stream_max_steps']==[3.]
    assert metrics['boundary_gradient_after'][0]<1e-5 and metrics['stream_max_gradient'][0]<1e-5
    result=dict(status='FL2A_BOUNDARY_ADAPTER_SYNTHETIC_PASS',complete_receiver_ready=False,
       supporting_checks=651,release_byte_identical=True,focused_sanitizer_findings=False,
       protected_files_unchanged=len(saved),metrics=metrics,
       not_done=['source ID lifecycle/transport integration','real journal replay',
                 'live experiment','performance benchmark','default promotion'],
       sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [
         *HERE.glob('*.h'),*HERE.glob('*.cpp'),ROOT/'build_release3',ROOT/'build_sanitize1',
         *[ROOT/(n+'.log') for n in logs]]})
    with (ROOT/'validation.json').open('x') as f:json.dump(result,f,indent=2)
    print(json.dumps({k:v for k,v in result.items() if k not in ('metrics','sha256')},indent=2))
if __name__=='__main__':main()
