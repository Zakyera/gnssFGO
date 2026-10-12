"""Offline symbolization of validated CPU samples; run in the pinned container.

All executable mappings in this experiment are ET_DYN ELF files. Return PCs
are adjusted by one byte; the interrupted instruction PC is not adjusted.
The maps and ELF symbols belong to the actual sampled process, not the host.
"""
from pathlib import Path
import collections
import json
import struct
import subprocess
import sys

ROOT = Path('/home/yeranis/results/gnssfgo_receiver_sampling_s5c_20261013')

def records(directory):
    start = json.loads((directory/'start.json').read_text())
    data = (directory/'samples.bin').read_bytes()
    if start['record_size'] == 560:
        return list(struct.Struct('<QQQQiiii64Q').iter_unpack(data))
    result = []; offset = 0
    while offset < len(data):
        header = struct.unpack_from('<QQQQiiii', data, offset); offset += 48
        depth = header[4]
        assert 0 < depth <= start['maximum_frames']
        pcs = struct.unpack_from('<'+'Q'*depth, data, offset); offset += 8*depth
        result.append((*header, *pcs))
    assert offset == len(data)
    return result

def main():
    tag = sys.argv[1]; directory = ROOT/(tag+'_profile')
    censored = len(sys.argv)>2 and sys.argv[2]=='censored'
    output = ROOT/(tag+('_censored' if censored else '')+'_symbols.json')
    assert not output.exists(), 'Refuse overwrite'
    quality=json.loads((ROOT/(tag+'_sampling_validation.json')).read_text())
    if censored:
        assert not quality['passed'] and quality['truncated']>0
        assert .90<=quality['coverage_ratio']<=1.05
        assert not any(quality[k] for k in ['write_failures','overruns','unwind_errors','empty_stacks'])
        assert quality['handler_unchanged'] and quality['main_tid_only']
        assert json.loads((ROOT/(tag+'_validation.json')).read_text())['passed']
    else:assert quality['passed']
    maps = []
    for line in (directory/'maps_end.txt').read_text().splitlines():
        fields = line.split(maxsplit=5)
        if len(fields) == 6 and 'x' in fields[1] and fields[5].startswith('/'):
            begin, end = [int(x,16) for x in fields[0].split('-')]
            maps.append((begin,end,int(fields[2],16),fields[5]))
    addresses = set()
    for row in records(directory):
        addresses.update(pc - (i > 0) for i, pc in enumerate(row[8:8+row[4]]) if pc)
    per_elf = collections.defaultdict(set); lookup = {}
    for pc in addresses:
        found = [(path,pc-begin+offset) for begin,end,offset,path in maps if begin<=pc<end]
        assert len(found) <= 1
        if found:
            path, relative = found[0]; per_elf[path].add(relative)
            lookup[pc] = (path,relative)
    symbols = {}
    for path, addresses_elf in per_elf.items():
        header = subprocess.check_output(['readelf','-h',path],text=True)
        assert 'DYN (Shared object file)' in header or 'DYN (Position-Independent Executable file)' in header, path
        order = sorted(addresses_elf)
        process = subprocess.run(['addr2line','-f','-C','-e',path],
            input=''.join(f'{v:x}\n' for v in order),capture_output=True,text=True,check=True)
        lines = process.stdout.splitlines(); assert len(lines) == len(order)*2
        symbols[path] = {v:dict(function=lines[2*i],location=lines[2*i+1]) for i,v in enumerate(order)}
    result = {}
    for pc in sorted(addresses):
        if pc in lookup:
            path,relative = lookup[pc]
            result[str(pc)] = dict(elf=path,address=relative,**symbols[path][relative])
        else:
            result[str(pc)] = dict(elf='[unmapped]',address=pc,function='??',location='??:0')
    with output.open('x') as f: json.dump(result,f)
    counts = collections.Counter()
    contexts = collections.Counter()
    for row in records(directory):
        names = [result[str(pc-(i>0))]['function'] for i,pc in enumerate(row[8:8+row[4]]) if pc]
        counts[names[0]] += 1
        contexts.update(set(names))
    print(json.dumps(dict(samples=len(records(directory)),unique_pcs=len(addresses),
        exclusive=[(n[:220],c) for n,c in counts.most_common(25)],
        inclusive=[(n[:220],c) for n,c in contexts.most_common(35)]),indent=2))

if __name__ == '__main__': main()
