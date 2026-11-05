import argparse,json,subprocess,time
from pathlib import Path
import numpy as np
import rerun as rr
from build_scene import H,LABELS,load,strips,sha
parser=argparse.ArgumentParser();parser.add_argument('--open',action='store_true');parser.add_argument('--screenshot',action='store_true');parser.add_argument('--receipt-suffix',choices=['','_host'],default='');args=parser.parse_args()
dest=H/'leica2_incremental_comparison.rrd'
if not args.open and not args.screenshot:
    check=json.loads((H/'scene_build_checks.json').read_text())
    products,cloud=load();expected={'/scene/environment/Velodyne_saved_ON_map':cloud};edges={}
    for (mode,role),(p,t) in products.items():
        entity=f'/scene/{mode}/{LABELS[role]}';expected[entity+'/samples']=p;edges[entity]=strips(p,t,role)
    seen=set();seenlines=set()
    for ch in rr.recording.load_recording(dest).chunks():
        entity=str(ch.entity_path);batch=ch.to_record_batch()
        if entity in expected:
            p=batch.column('Points3D:positions').values.values.to_numpy().reshape(-1,3)
            assert np.array_equal(p,expected[entity]),entity
            seen.add(entity)
        if entity in edges:
            lines=batch.column('LineStrips3D:strips')[0].as_py();assert len(lines)==len(edges[entity])
            for p,q in zip(lines,edges[entity]):assert np.array_equal(np.asarray(p,np.float32),q),entity
            seenlines.add(entity)
    assert seen==set(expected) and seenlines==set(edges)
    assert all(sha(p)==d for p,d in check['input_sha256'].items())
    result={'pass_':True,'all_vertices_and_edges_exact':True,'cloud_points_exact':len(cloud),'inputs_unchanged':True,'rrd_sha256':sha(dest),'bytes':dest.stat().st_size}
    with (H/'scene_recording_checks.json').open('x') as f:json.dump(result,f,indent=2)
else:
    assert json.loads((H/'scene_recording_checks.json').read_text())['pass_']
    command=['/home/yeranis/.local/bin/rerun',str(dest),'--bind','127.0.0.1','--port','auto','--memory-limit','2GB','--window-size','1600x1100']
    label=('screenshot' if args.screenshot else 'viewer')+args.receipt_suffix
    assert not (H/(label+'_launch.json')).exists()
    if args.screenshot:command+=['--screenshot-to',str(H/'rerun_preview.png')]
    with (H/(label+'.log')).open('x') as log:p=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    result={'pid':p.pid,'command':command,'time':time.time()}
    with (H/(label+'_launch.json')).open('x') as f:json.dump(result,f,indent=2)
    if args.screenshot:
        code=p.wait(timeout=60);result['returncode']=code
        result['image_written']=(H/'rerun_preview.png').is_file()
        if result['image_written']:result['image_sha256']=sha(H/'rerun_preview.png')
        with (H/(label+'_result.json')).open('x') as f:json.dump(result,f,indent=2)
        assert result['image_written'],'No screenshot; inspect retained log'
    else:
        time.sleep(3);assert p.poll() is None
print(json.dumps(result,indent=2))
