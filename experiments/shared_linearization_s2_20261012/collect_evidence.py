"""Summarize retained S2 checks without executing an estimator or editing sources."""
from pathlib import Path
import hashlib
import json
import re
import shutil
import xml.etree.ElementTree as ET

HERE=Path(__file__).resolve().parent
ROOT=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/shared_linearization_s2_20261012')
BUILDS=['release_6','asan_2','ubsan_2']
SOURCES=['NativeGaussianHistory.h','NativeGaussianHistory.cpp','test_native_gaussian.cpp',
         'test_main_cache.cpp','build.py','run_tests.py']

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    out=HERE/'evidence';out.mkdir(exist_ok=False)
    result={'builds':{},'source_sha256':{s:sha(HERE/s) for s in SOURCES}}
    for build in BUILDS:
        path=ROOT/build;dest=out/build;dest.mkdir()
        for s in SOURCES:assert sha(path/'source'/s)==sha(HERE/s),(build,s)
        for name in ['build.log','build_result.json','commands.json']:
            shutil.copy2(path/name,dest/name)
        shutil.copytree(path/'checks1',dest/'checks1')
        tests=ET.parse(path/'checks1/tests.xml').getroot()
        assert tests.get('failures')=='0' and tests.get('errors')=='0'
        log=(path/'checks1/tests.log').read_text()
        numeric={}
        for name,absolute,relative in re.findall(r'^NUMERIC (\S+) abs=(\S+) relative=(\S+)',log,re.M):
            old=numeric.setdefault(name,{'max_absolute':0.,'max_relative':0.})
            old['max_absolute']=max(old['max_absolute'],float(absolute))
            old['max_relative']=max(old['max_relative'],float(relative))
        run=json.loads((path/'checks1/result.json').read_text());assert run['returncode']==0
        assert run['gtsam_sha256']=='369f45866a82c40170ad7cfa83b4bc6b7f8a6f4ae22db34115daee926529bb37'
        result['builds'][build]={'tests':int(tests.get('tests')),'numeric':numeric,'run':run}
    first=(ROOT/'release_6/checks1/tests.log').read_text()
    second=(ROOT/'release_6/checks2/tests.log').read_text()
    meaningful=lambda s:[l for l in s.splitlines() if l.startswith(('NUMERIC ','CONDITION ','BOUND ','MAIN_CACHE '))]
    assert meaningful(first)==meaningful(second),'repeated numerical diagnostics differ'
    assert ET.parse(ROOT/'release_6/checks2/tests.xml').getroot().get('failures')=='0'
    shutil.copytree(ROOT/'release_6/checks2',out/'release_6/checks2')
    result['deterministic_numerical_diagnostics_exact']=True
    result['fixture_condition_max']=max(float(v) for v in re.findall(r'^CONDITION (\S+)',first,re.M))
    (out/'summary.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2))
if __name__=='__main__':main()
