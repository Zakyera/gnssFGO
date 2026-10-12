"""Summarize only completed correctness checks; never starts a timing test."""
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/e1b4a_local_update_batching_20261012')
sets={
    'release':ROOT/'release_2/checks3',
    'ubsan':ROOT/'ubsan_1/checks1',
    'asan_ubsan_cpu':ROOT/'asan_2/checks1',
}
summary={'timing_test_started':False,'checks':{}}
for name,path in sets.items():
    assert json.loads((path/'passed.json').read_text())['passed']
    cases=[];maxima=[0.0]*5;matrix_max=0.0
    for xml in path.glob('*.xml'):
        root=ET.parse(xml).getroot()
        assert root.get('failures')=='0' and root.get('errors')=='0'
        cases += [c.get('classname')+'.'+c.get('name') for c in root.iter('testcase') if c.get('status')=='run']
    for log in path.glob('*.log'):
        for line in log.read_text().splitlines():
            if line.startswith('E1B4_STATE_MAX,'):
                maxima=[max(x,float(y)) for x,y in zip(maxima,line.split(',')[1:])]
            if line.startswith('E1_ERROR '):
                matrix_max=max(matrix_max,float(line.split('max_abs=',1)[1].split()[0]))
    summary['checks'][name]=dict(passed=len(cases),tests=cases,
        maximum_p_orientation_v_bias_covariance_differences=maxima,
        maximum_printed_matrix_difference=matrix_max)
for log in ROOT.glob('*/checks*/*.log'):
    assert 'E1B4_TIMING,' not in log.read_text(),'timing unexpectedly started'
summary['artifacts']={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest()
    for p in [ROOT/'release_2/libgtsam.so.4.3a1',ROOT/'release_2/libglim_ros.so',
              ROOT/'release_2/test_dispatch_3',ROOT/'release_2/test_solver_3']}
with (ROOT/'correctness_summary.json').open('x') as f:json.dump(summary,f,indent=2);f.write('\n')
print(json.dumps({k:dict(passed=v['passed'],maxima=v['maximum_p_orientation_v_bias_covariance_differences'],matrix_max=v['maximum_printed_matrix_difference']) for k,v in summary['checks'].items()},indent=2))
