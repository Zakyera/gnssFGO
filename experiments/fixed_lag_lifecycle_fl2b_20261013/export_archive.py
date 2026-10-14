"""Postprocess committed receiver output; no optimization, GT or alignment.

Explicitly separate online/as-played from fixed-lag finalized (NOT global final).
Refuse incomplete archives and existing destinations. Never edit raw commits.
"""
import csv,json,sys
from pathlib import Path

def export(root,out):
    streams={k:[] for k in ('online','finalized','drain','publication','belief','absorbed','end')}
    for bundle in sorted((root/'commits').iterdir(),key=lambda p:int(p.name)):
        with (bundle/'records.txt').open() as f:
            for row in csv.reader(f):
                if row:streams[row[0]].append(row)
    assert len(streams['end'])==1,'missing/duplicate successful end marker'
    assert len(streams['drain'])==1
    for kind in ('online','finalized'):
        rows=streams[kind];indices=[int(r[1]) for r in rows]
        assert indices==list(range(len(rows))),f'{kind} duplicate/missing/out-of-order support'
        assert all(int(r[3])>=int(r[2]) for r in rows)
    assert len(streams['online'])==len(streams['finalized'])
    assert [r[2] for r in streams['online']]==[r[2] for r in streams['finalized']]
    assert all(float(r[3])<=1e-5 for r in streams['publication'])
    out.mkdir(exist_ok=False)
    for kind in ('online','finalized'):
        with (out/(kind+'.tum')).open('x') as f:
            for r in streams[kind]:
                ns=int(r[2]);stamp=f'{ns//1000000000}.{ns%1000000000:09d}'
                f.write(stamp+' '+' '.join(r[4:11])+'\n')
        with (out/(kind+'_states.csv')).open('x') as f:
            writer=csv.writer(f);writer.writerow(['kind','index','physical_ns','available_ns','px','py','pz','qx','qy','qz','qw','vx','vy','vz','wx','wy','wz']);writer.writerows(streams[kind])
    with (out/'summary.json').open('x') as f:
        json.dump(dict(counts={k:len(v) for k,v in streams.items()},
          online='immutable support publications',
          finalized='outgoing validated states at retirement, plus remaining EOF window; not full-history retrospective',
          gt_used=False,model_rerun=False),f,indent=2)
    return streams
if __name__=='__main__':export(Path(sys.argv[1]),Path(sys.argv[2]))
