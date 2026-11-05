"""Read only the initial/theta values in CP51's explicit native binary archive."""
import hashlib
import json
from pathlib import Path
import struct

H=Path(__file__).resolve().parent
p=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/eig2_hesai_insertion_cpv51_20261030/prepared/full_on_source/checkpoints/update_6/after_ordinary/checkpoint.bin')
b=p.read_bytes();off=0
def u64():
    global off
    n=struct.unpack_from('<Q',b,off)[0];off+=8;return n
def matrix():
    global off
    r=u64();c=u64();assert r*c<1000
    off+=8*r*c
    return [r,c]
length=u64();magic=b[off:off+length];off+=length
assert magic==b'CPV36_2_NATIVE_X86_64'
parts=[];fields=[]
for name in ['theta','initial']:
    start=off;n=u64();keys=[]
    for i in range(n):
        k=u64();ch=chr(k>>56);idx=k&((1<<56)-1)
        assert ch in 'xvb'
        shapes=[matrix() for _ in range(1 if ch=='v' else 2)]
        assert shapes==({'x':[[3,3],[3,1]],'v':[[3,1]],'b':[[3,1],[3,1]]}[ch])
        keys.append(ch+str(idx))
    parts.append(b[start:off]);fields.append(dict(name=name,offset=start,length=off-start,keys=keys,sha256=hashlib.sha256(parts[-1]).hexdigest()))
assert parts[0]==parts[1], 'genuinely different serialized input values'
record=dict(byte_identical=True,input=str(p),input_sha256=hashlib.sha256(b).hexdigest(),fields=fields,
            prior_CP52_tangent_residual=2.6762126660222371e-19,
            conclusion='Exact same saved matrices/vectors; exact-zero localCoordinates gate fails on floating-point group arithmetic, not distinct initial states.')
with (H/'initial_byte_identity.json').open('x') as f:json.dump(record,f,indent=2)
print(json.dumps(record,indent=2))
