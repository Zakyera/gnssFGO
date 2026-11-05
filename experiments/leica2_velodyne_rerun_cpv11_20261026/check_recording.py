"""Read the saved RRD back and verify every temporal native point and segment."""
import hashlib
import json
from pathlib import Path
import numpy as np
import pyarrow as pa
import rerun as rr
from render import COLORS, NAMES

H=Path(__file__).resolve().parent
z=np.load(H/'trajectories.npz')
recording=rr.recording.load_recording(H/'leica2_velodyne_first_published.rrd')
chunks=list(recording.chunks())
checks={}
for mode in ('off','on'):
    for role in COLORS:
        key='reference' if role=='reference' else mode+'_'+role
        times=z[key+'_t_ns']; pos=z[key+'_p'].astype(np.float32)
        entity='/detail/'+mode+'/'+NAMES[role]
        rows=[]; segments=[]
        for ch in chunks:
            name=str(ch.entity_path).replace('\\ ',' ')
            if ch.is_static or name not in (entity,entity+'/samples'):
                continue
            batch=ch.to_record_batch()
            t=batch.column('seconds').cast(pa.int64()).to_numpy()
            if name.endswith('/samples'):
                p=np.asarray(batch.column('Points3D:positions').to_pylist(),np.float32).reshape(-1,3)
                rows.append((t,p))
            else:
                p=np.asarray(batch.column('LineStrips3D:strips').to_pylist(),np.float32).reshape(-1,2,3)
                segments.append((t,p))
        t=np.concatenate([v[0] for v in rows]);p=np.concatenate([v[1] for v in rows]);order=np.argsort(t)
        assert np.array_equal(t[order],times), key
        assert np.array_equal(p[order],pos), key
        gap=50_000_000 if role=='reference' else 500_000_000
        ok=np.diff(times)<=gap
        t=np.concatenate([v[0] for v in segments]);p=np.concatenate([v[1] for v in segments]);order=np.argsort(t)
        assert np.array_equal(t[order],times[1:][ok]),key
        assert np.array_equal(p[order],np.stack([pos[:-1],pos[1:]],axis=1)[ok]),key
        checks[mode+'_'+role]={'points_exact':len(times),'segments_exact':int(ok.sum())}
meta=json.loads((H/'data_checks.json').read_text())
for path,digest in meta['input_sha256'].items():
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        while data:=f.read(8*1024*1024):h.update(data)
    assert h.hexdigest()==digest,path
result={'passed':True,'every_timestamp_and_float32_vertex_exact':True,
        'every_segment_and_gap_exact':True,'inputs_unchanged':True,'chunks':len(chunks),'products':checks}
with (H/'recording_checks.json').open('x') as f:json.dump(result,f,indent=2)
print(json.dumps(result,indent=2))
