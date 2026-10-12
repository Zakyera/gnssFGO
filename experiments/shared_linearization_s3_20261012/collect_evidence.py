from pathlib import Path
import hashlib
import json
import shutil
import xml.etree.ElementTree as ET

HERE=Path(__file__).resolve().parent
ROOT=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/shared_linearization_s3_20261012')

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    dest=HERE/'evidence';dest.mkdir(exist_ok=False)
    numbers=[];sources={};summary={}
    for name,checks in [('release_3',['checks1','checks2']),('asan_2',['checks1'])]:
        build=ROOT/name;out=dest/name;out.mkdir()
        for filename in ['commands.json','build_result.json','protected.json']:
            shutil.copy2(build/filename,out/filename)
        source_files=['ShadowObserver.cpp','ShadowObserver.h','test_lifecycle.cpp','build.py','control.cpp']
        for f in source_files:assert (build/'source'/f).read_bytes()==(HERE/f).read_bytes(),(name,f)
        for f in ['NativeGaussianHistory.cpp','NativeGaussianHistory.h','test_native_gaussian.cpp']:
            assert (build/'source'/f).read_bytes()==(HERE.parent/'shared_linearization_s2_20261012'/f).read_bytes(),f
        sources[name]={str(p.relative_to(build)):sha(p) for path in [build/'source',build/'overlay'] for p in path.rglob('*') if p.is_file()}
        for check in checks:
            shutil.copytree(build/check,out/check)
            xml=ET.parse(build/check/'tests.xml').getroot()
            assert xml.attrib['tests']=='42' and xml.attrib['failures']=='0' and xml.attrib['errors']=='0'
            result=json.loads((build/check/'result.json').read_text());assert result['returncode']==0
            lines=[x for x in (build/check/'tests.log').read_text().splitlines() if x.startswith(('NUMERIC ','CONDITION ','BOUND ','LIFECYCLE '))]
            numbers.append(lines);summary[name+'/'+check]={'tests':42,'passed':True}
    assert all(x==numbers[0] for x in numbers[1:]),'numeric diagnostics not deterministic'
    nonreg=dest/'nonregression';nonreg.mkdir()
    for filename in ['result.json','bridge_ldd.stdout','bridge_ldd.stderr','original_ldd.stdout','original_ldd.stderr',
                     'regression.xml','regression.stderr']:
        shutil.copy2(ROOT/'release_3/nonregression'/filename,nonreg/filename)
    summary['nonregression']=json.loads((nonreg/'result.json').read_text())
    summary['numeric_lines_exact_across_repeats_and_sanitizers']=True
    summary['S2_backend_and_tests_unchanged']=True
    (dest/'source_hashes.json').write_text(json.dumps(sources,indent=2))
    (dest/'numeric_summary.txt').write_text('\n'.join(numbers[0])+'\n')
    (dest/'summary.json').write_text(json.dumps(summary,indent=2))
    print(json.dumps(summary,indent=2))
if __name__=='__main__':main()
