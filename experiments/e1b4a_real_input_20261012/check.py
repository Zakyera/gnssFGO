"""Preflight/postflight of the one authorized diagnostic capture."""
import argparse
import ast
import json
import os
from pathlib import Path
import shutil
import subprocess
import xml.etree.ElementTree as ET
from prepare import ROOT,WS,HERE,BATCH,CHECK,BASE,PROFILE,sha,inside

def docker(args,label):
    r=subprocess.run(['docker','exec','cbsms_ws_gpu',*args],capture_output=True,text=True)
    with (ROOT/(label+'.log')).open('x') as f:f.write(r.stdout+r.stderr)
    assert r.returncode==0,(label,r.returncode,r.stderr[-2000:])
    return r.stdout
def integrity():
    protected=json.loads((ROOT/'protected.json').read_text())
    for p,h in protected.items():assert sha(p)==h,p
    repos=json.loads((ROOT/'source_git.json').read_text())
    for p,d in repos.items():
        assert subprocess.check_output(['git','-C',p,'rev-parse','HEAD'],text=True).strip()==d['head']
        assert not subprocess.check_output(['git','-C',p,'status','--porcelain'],text=True).strip(),p
    return dict(protected_hashes=len(protected),clean_repos=len(repos))
def preflight():
    assert not (ROOT/'preflight.json').exists()
    assert shutil.disk_usage(ROOT).free>10*1024**3
    frozen=integrity()
    for p in ROOT.glob('*.py'):ast.parse(p.read_text())
    for p in list(ROOT.glob('*.sh'))+list(ROOT.glob('*.env')):subprocess.run(['bash','-n',str(p)],check=True)
    for p in ROOT.glob('*.launch'):ET.parse(p)
    assert json.loads((ROOT/'release_1/complete.json').read_text())['passed']
    docker(['python3','-c',"import socket; s=socket.socket(); assert s.connect_ex(('127.0.0.1',11413))!=0"],'port')
    processes=docker(['ps','-eo','pid,args'],'processes')
    for line in processes.splitlines():
        assert not any(x in line for x in ['/devel/lib/glim_ros/glim_rosnode','/devel/lib/kimera_vio_ros/kimera_vio_ros_node','rosbag play']),line
    docker(['nvidia-smi','--query-gpu=name,driver_version,memory.used','--format=csv'],'gpu')
    setup='source /opt/ros/noetic/setup.bash && source /workspace/cbs_gtsam4.3/devel/setup.bash && '
    env='export LD_LIBRARY_PATH='+inside(ROOT/'release_1')+':$LD_LIBRARY_PATH && '
    linked=docker(['bash','-lc',setup+env+'ldd -r '+inside(ROOT/'release_1/libglim_ros.so')],'ldd')
    assert 'not found' not in linked and 'undefined symbol' not in linked
    # Same frozen checker fixtures against the exact runtime dependency chain.
    result=docker(['bash','-lc',setup+env+inside(CHECK/'release_5/test_snapshot')+
       ' --gtest_output=xml:'+inside(ROOT/'regression.xml')],'regression')
    tree=ET.parse(ROOT/'regression.xml').getroot()
    assert int(tree.attrib['failures'])==0 and int(tree.attrib['tests'])-int(tree.attrib.get('disabled','0'))==40,tree.attrib
    bags={}
    for suffix in ['hesai','cpt7_imu','alphasense']:
        p=WS/'src/datasets/GrandTour'/('2024-10-01-11-29-55_'+suffix+'.bag')
        s=p.stat();bags[str(p)]=dict(path=str(p.resolve()),bytes=s.st_size,mtime_ns=s.st_mtime_ns)
        docker(['bash','-lc',setup+'rosbag info --yaml '+inside(p)],'bag_'+suffix)
    # Check generated sample scope, no stale previous observer and no profiler.
    assert 'RealGraphAudit' not in (ROOT/'authoring/cbs_bridge.cpp').read_text()
    assert 'CBS_E1B_PROFILE_PATH' not in (ROOT/'source.env').read_text()
    pins={str(p):sha(p) for p in ROOT.rglob('*') if (p.is_file() or p.is_symlink()) and
          (p.suffix in ['.py','.sh','.env','.launch','.h','.cpp','.so'] or '.so.' in p.name)}
    with (ROOT/'preflight.json').open('x') as f:json.dump(dict(passed=True,integrity=frozen,regression=tree.attrib,
        bags_metadata_not_full_hashes=bags,prepared_hashes=pins),f,indent=2)
    print('REAL_INPUT_PREFLIGHT_PASS',frozen,flush=True)
def validate():
    assert not (ROOT/'audit/FAILED.txt').exists()
    complete=json.loads((ROOT/'control/complete.json').read_text());assert complete['passed']
    source=ROOT/'e1_source';assert (source/'complete.marker').exists()
    effective=json.loads((source/'effective_mode_check.json').read_text())
    assert effective['passed'] and effective['checked_before_playback']
    coverage=json.loads((source/'source_coverage.json').read_text());assert coverage['passed']
    samples=[json.loads((ROOT/'audit'/('sample_%d.json'%n)).read_text()) for n in [10,30,60,100,150,180]]
    assert all(s['passed'] and s['batched_graphs']>0 and s['prepared_factors']>0 for s in samples)
    events=[json.loads(l) for l in (ROOT/'audit/updates.jsonl').read_text().splitlines()]
    assert [r['call'] for r in events]==list(range(1,len(events)+1))
    retired=[r for r in events if r['retired_keys']>0];assert retired
    assert sum(s['call']>=retired[0]['call'] for s in samples)>=3
    for section in ['paired','fidelity']:
        for s in samples:
            d=s[section]
            for k in ['position','velocity','bias']:assert 0<=d[k]<=1e-9,(s,k)
            assert 0<=d['orientation']<=1e-10
    baseline=WS/'runs/grandtour_eth1_presentation/grandtour_eth1_stereo_shared_cpt7_no_sidecar_cbs_off_full_346s_20260907'
    for name in ['glim_config','kimera_profile']:
        def hashes(d):return {str(p.relative_to(d)):sha(p) for p in d.rglob('*') if p.is_file()}
        assert hashes(baseline/name)==hashes(source/name),name
    pre=json.loads((ROOT/'preflight.json').read_text())
    for p,h in pre['prepared_hashes'].items():assert sha(p)==h,p
    counts={k:0 for k in ['GLIM_CBS_ACTIVE_WINDOW_FACTOR_ROW,active_window_remove_committed',
                         'GLIM_CBS_ODOM_INJECT_ROW','active_window_temporary_odom_applied']}
    for p in (source/'ros_logs').rglob('*stdout.log'):
        if p.name.startswith('rosout'):continue
        with p.open(errors='replace') as f:
            for line in f:
                for token in counts:
                    if token in line:counts[token]+=1
    assert all(counts.values()),counts
    result=dict(passed=True,integrity=integrity(),complete=complete,samples=samples,coverage=coverage,
        updates=len(events),first_retirement=retired[0],retirement_updates=len(retired),
        retired_keys=sum(r['retired_keys'] for r in retired),source_lifecycle=counts,
        no_receiver=True,not_throughput_acceptance=True)
    with (ROOT/'validation.json').open('x') as f:json.dump(result,f,indent=2)
    print(json.dumps(result,indent=2))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('stage',choices=['preflight','validate','integrity']);a=p.parse_args()
    if a.stage=='preflight':preflight()
    elif a.stage=='validate':validate()
    else:print(json.dumps(integrity(),indent=2))
