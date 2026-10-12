"""Summarize retained test evidence, not a new estimator experiment."""
from pathlib import Path
import hashlib
import json
import xml.etree.ElementTree as ET
from prepare import ROOT

HERE=Path(__file__).resolve().parent
def main():
    cases={}
    for variant,folder in [('release_5','final'),('ubsan_2','final'),('asan_2','cpu_final')]:
        p=ROOT/variant/folder;r=json.loads((p/'result.json').read_text())
        assert r['returncode']==0
        xml=ET.parse(p/'tests.xml').getroot()
        tests=[x for x in xml.iter('testcase') if x.attrib.get('status')=='run']
        assert all(x.find('failure') is None for x in tests)
        rows=[[float(x) for x in line.split(',')[1:]] for line in (p/'test.log').read_text().splitlines()
              if line.startswith('SNAPSHOT_SOLVER,')]
        if variant!='asan_2':assert rows[-1]==rows[-2], 'repeated fixture not deterministic'
        cases[variant]=dict(tests_run=len(tests),failures=0,executable_sha256=r['executable_sha256'],
                           solver_rows=rows,tests=[x.attrib['classname']+'.'+x.attrib['name'] for x in tests])
    # Sanitizer instrumentation changes compiler arithmetic. Compare its
    # measured differences to the declared gates, not bitwise rotation logs.
    assert [r[:4] for r in cases['release_5']['solver_rows']]==[r[:4] for r in cases['ubsan_2']['solver_rows']]
    all_rows=[r for v in cases.values() for r in v['solver_rows']]
    maximum=[max(row[i] for row in all_rows) for i in range(4,11)]
    assert maximum[0]<=1e-9 and maximum[1]<=1e-10
    assert maximum[2]<=1e-9 and maximum[3]<=1e-9
    outputs=dict(passed=True,tests=cases,max_differences=dict(zip(
        ['position','orientation_rad','velocity','bias','gaussian','full_covariance','relative_covariance'],maximum)),
        integrity=json.loads((ROOT/'integrity_final.json').read_text()))
    for name in ['ExactGpuSnapshot.h','ExactGpuSnapshot.cpp','LocalUpdateCheck.h','test_snapshot.cpp']:
        original=(HERE/name).read_bytes()
        for variant in cases:assert original==(ROOT/variant/'source'/name).read_bytes(),(variant,name)
    outputs['diagnostic_source_sha256']={name:hashlib.sha256((HERE/name).read_bytes()).hexdigest()
        for name in ['ExactGpuSnapshot.h','ExactGpuSnapshot.cpp','LocalUpdateCheck.h','test_snapshot.cpp']}
    for p in [ROOT/'summary.json',HERE/'summary.json']:
        with p.open('x') as f:json.dump(outputs,f,indent=2);f.write('\n')
    print(json.dumps(outputs['max_differences'],indent=2))
    print({v:x['tests_run'] for v,x in cases.items()})
if __name__=='__main__':main()
