"""Exact native RRD readback: every sample, gap-aware edge and map point."""
import json
from pathlib import Path
import numpy as np
import rerun as rr
from build import H,OUT,LABELS,pieces,sha


def verify(case):
    z = np.load(H/(case+'.npz'))
    cloud = np.load(H/(case+'_cloud.npz'))['position']
    dest = OUT/(case+'_method2.rrd')
    expected = {'/scene/environment/Saved_ON_GLiM_map_context':cloud}
    edges = {}
    for mode in ('off','on'):
        for role,label in LABELS.items():
            key = 'reference' if role=='reference' else mode+'_'+role
            ent = f'/scene/{mode}/{label}'
            p,t = z[key+'_p'].astype(np.float32),z[key+'_t_ns']
            expected[ent+'/samples'] = p
            edges[ent] = pieces(p,t,role)
    seen, seenedges, notes = set(),set(),set()
    for chunk in rr.recording.load_recording(dest).chunks():
        entity = str(chunk.entity_path)
        batch = chunk.to_record_batch()
        if entity in expected:
            points = batch.column('Points3D:positions').values.values.to_numpy().reshape(-1,3)
            assert np.array_equal(points,expected[entity]),(case,entity)
            assert entity not in seen
            seen.add(entity)
        if entity in edges:
            strips = batch.column('LineStrips3D:strips')[0].as_py()
            assert len(strips)==len(edges[entity]),(case,entity)
            for p,q in zip(strips,edges[entity]):
                assert np.array_equal(np.asarray(p,np.float32),q),(case,entity)
            assert entity not in seenedges
            seenedges.add(entity)
        if entity in ('/notes','/legend'): notes.add(entity)
    assert seen == set(expected) and seenedges == set(edges)
    assert notes == {'/notes','/legend'}
    return dict(case=case,passed=True,rrd=str(dest),rrd_sha256=sha(dest),
        rrd_bytes=dest.stat().st_size,cloud_points=len(cloud),
        sample_points=sum(len(p) for ent,p in expected.items() if ent.endswith('/samples')),
        path_edges=sum(len(p)-1 for lines in edges.values() for p in lines),
        every_vertex_edge_cloud_point_exact=True)


if __name__=='__main__':
    inp = json.loads((H/'input_checks.json').read_text())
    results = []
    for case in inp['cases']:
        result = verify(case)
        results.append(result)
        print(json.dumps(result),flush=True)
    assert all(sha(p)==digest for p,digest in inp['input_sha256'].items())
    for case in inp['cases']:
        meta = json.loads((H/(case+'.json')).read_text())
        assert sha(H/(case+'.npz')) == meta['trajectory_npz_sha256']
        assert sha(H/(case+'_cloud.npz')) == meta['cloud_npz_sha256']
    with (H/'verification.json').open('x') as f:
        json.dump(dict(passed=True,cases=results,all_inputs_unchanged=True,
            input_count=len(inp['input_sha256']),rerun_version=rr.__version__),f,indent=2)
