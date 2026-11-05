"""Verify all full-overlay vertices/edges and open the corrected Rerun scene."""
import argparse
import json
import subprocess
import time
from datetime import datetime
from pathlib import Path
import numpy as np
import rerun as rr
from build import LABELS, sha

H=Path(__file__).resolve().parent
parser=argparse.ArgumentParser();parser.add_argument('--open',action='store_true');args=parser.parse_args()
dest=H/'leica2_hesai_full_scene.rrd'
if not args.open:
    check=json.loads((H/'scene_build_checks.json').read_text())
    original=json.loads((H/'data_checks.json').read_text())
    pins={**original['input_sha256'],**check['input_sha256']}
    z=np.load(H/'trajectories.npz')
    cloud=np.load(H/'context_cloud.npz')['position']
    expected={'/scene/environment/Hesai_saved_ON_map':cloud}
    edges={}
    for mode in ('off','on'):
        for role,label in LABELS.items():
            key='reference' if role=='reference' else mode+'_'+role
            entity=f'/scene/{mode}/{label}'
            p=z[key+'_p'].astype(np.float32);t=z[key+'_t_ns']
            expected[entity+'/samples']=p
            gap=50_000_000 if role=='reference' else 500_000_000
            pieces=np.split(np.arange(len(p)),np.flatnonzero(np.diff(t)>gap)+1)
            edges[entity]=[p[ix] for ix in pieces if len(ix)>1]
    seen=set();seenedges=set()
    for ch in rr.recording.load_recording(dest).chunks():
        entity=str(ch.entity_path);batch=ch.to_record_batch()
        if entity in expected:
            points=batch.column('Points3D:positions').values.values.to_numpy().reshape(-1,3)
            assert np.array_equal(points,expected[entity]),entity
            seen.add(entity)
        if entity in edges:
            strips=batch.column('LineStrips3D:strips')[0].as_py()
            assert len(strips)==len(edges[entity]),entity
            for p,q in zip(strips,edges[entity]):assert np.array_equal(np.asarray(p,np.float32),q),entity
            seenedges.add(entity)
    assert seen==set(expected) and seenedges==set(edges)
    assert all(sha(Path(p))==v for p,v in pins.items())
    result={'passed':True,'all_cloud_points_exact':True,'cloud_points':len(cloud),'all_full_trajectory_vertices_and_edges_exact':True,
        'all_inputs_unchanged':True,'input_count':len(pins),'input_sha256':pins,
        'rrd_sha256':sha(dest),'rrd_bytes':dest.stat().st_size}
    with (H/'scene_recording_checks.json').open('x') as f:json.dump(result,f,indent=2)
    print(json.dumps(result,indent=2))
else:
    verified=json.loads((H/'scene_recording_checks.json').read_text())
    assert verified['passed'] and sha(dest)==verified['rrd_sha256']
    assert not (H/'scene_viewer_launch.json').exists()
    cmd=['/home/yeranis/.local/bin/rerun',str(dest),'--bind','127.0.0.1','--port','auto',
         '--memory-limit','2GB','--window-size','1600x1100']
    with (H/'scene_viewer.log').open('x') as log:
        p=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    time.sleep(3)
    result={'pid':p.pid,'command':cmd,'started':datetime.now().astimezone().isoformat(),'exit_after_3s':p.poll()}
    with (H/'scene_viewer_launch.json').open('x') as f:json.dump(result,f,indent=2)
    print(json.dumps(result,indent=2))
    assert result['exit_after_3s'] is None
