"""Verify sampling cadence/stack completion before interpreting hotspots."""
from pathlib import Path
import json
import struct
import statistics
import sys
ROOT=Path('/home/yeranis/results/gnssfgo_receiver_sampling_s5c_20261013')
tag=sys.argv[1];directory=ROOT/(tag+'_profile')
start=json.loads((directory/'start.json').read_text());end=json.loads((directory/'end.json').read_text())
data=(directory/'samples.bin').read_bytes()
if start['record_size']==560:
    record=struct.Struct('<QQQQiiii64Q');assert len(data)%record.size==0
    samples=list(record.iter_unpack(data))
else:
    samples=[];o=0
    while o<len(data):
        r=struct.unpack_from('<QQQQiiii',data,o);o+=48
        assert 0<r[4]<=start['maximum_frames']
        pcs=struct.unpack_from('<'+'Q'*r[4],data,o);o+=8*r[4]
        samples.append((*r,*pcs))
    assert o==len(data)
assert len(samples)==end['samples']
cpu=end['thread_cpu_ns']; expected=cpu/start['interval_ns'];ratio=len(samples)/expected
gaps=[(b[2]-a[2])/1e6 for a,b in zip(samples,samples[1:])]
result=dict(expected_samples=expected,coverage_ratio=ratio,
  median_cpu_gap_ms=statistics.median(gaps),maximum_cpu_gap_ms=max(gaps),
  unwind_errors=sum(s[5]<0 for s in samples),empty_stacks=sum(s[4]==0 for s in samples),
  median_depth=statistics.median(s[4] for s in samples),
  unwind_cpu_fraction=sum(s[3] for s in samples)/cpu,
  main_tid_only=all(s[7]==start['tid'] for s in samples),**end)
result['passed']=.90<=ratio<=1.05 and not any([end['write_failures'],end['overruns'],end['truncated'],result['unwind_errors'],result['empty_stacks']]) and end['handler_unchanged'] and result['main_tid_only']
output=ROOT/(tag+'_sampling_validation.json');assert not output.exists()
with output.open('x') as f:json.dump(result,f,indent=2)
print(json.dumps(result,indent=2))
if not result['passed']:raise SystemExit(1)
