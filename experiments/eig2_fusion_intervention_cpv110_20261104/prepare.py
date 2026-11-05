"""Input-only preparation, guards and exact causal anchors. No receiver launch."""
import collections,csv,importlib.util,shutil,sys
import numpy as np
from common import *
sys.dont_write_bytecode=True
assert not (H/'prepared.json').exists()
pins={}
for root,want in ((S,383),(D,None)):
    complete=read(root/'complete.json');assert complete['passed'];n=check(complete['output_sha256'])
    if want:assert n==want
    pins[str(root/'complete.json')]=sha(root/'complete.json')
assert check(read(P/'protected_manifest.json'))==8873
cfg=read(R/'resolved_config.json');epoch=read(P/'input_audit.json')['epoch_ns']
assert cfg['receiver_model']=='source_wnoa' and cfg['receiver_fixed_lag_seconds']==7
assert cfg['imu_frame']=='cpt7_imu' and all(x['belief_body_frame']=='cpt7_imu' for x in cfg['sources'])
assert sha(B)=='4b08a7f5ba97f5e4ec22959df23367df79eebf5d644dc19d939747dcd4566d6e'
assert shutil.disk_usage(H).free>25*2**30
for p in [B,P/'receiver_cpk19.sh',R/'resolved_config.json',R/'events.csv',P/'input_audit.json',H/'PROTOCOL.txt',H/'common.py',Path(__file__),E/'receiver_finalization_cpk19_20261021/Receiver.h',E/'receiver_finalization_cpk19_20261021/Live.cpp']:
    pins[str(p)]=sha(p)
def eligible(start,end):
    a,b=(start-epoch)*1e-9,(end-epoch)*1e-9
    return any(lo<=a<b<=hi for lo,hi in WINDOWS)
anchors={};target_ids=set()
for directory in sorted((R/'receiver/archive/commits').iterdir(),key=lambda p:int(p.name)):
    with (directory/'records.txt').open() as f:
        for row in csv.reader(f):
            if row[:2] not in (['belief','insert'],['belief','replace']) or row[2]!='kimera_front':continue
            assert len(row)==55
            start,end,arrival=map(int,row[8:11]);key=tuple(row[2:6])
            if not eligible(start,end):continue
            target_ids.add(key)
            if arrival-end<=2_500_000_000:
                anchors[key]=dict(revision=int(row[6]),ordinal=int(row[7]),start=start,end=end,arrival=arrival,commit=directory.name,mean=row[13:19],covariance=row[19:])
assert anchors.keys()==target_ids and 250<len(anchors)<300,len(anchors)
assert all(2.15<=(x['arrival']-x['end'])*1e-9<=2.5 for x in anchors.values())
gpath=E/'fixed_lag_eig2_g2k_reference_fl4y_20261017/analyze.py';pins[str(gpath)]=sha(gpath)
spec=importlib.util.spec_from_file_location('geometry',gpath);g=importlib.util.module_from_spec(spec);spec.loader.exec_module(g);assert g.self_test()
for x in anchors.values():
    C=np.array(x['covariance'],float).reshape(6,6);assert np.linalg.eigvalsh(C).min()>0 and np.max(abs(C-C.T))<1e-12
    assert np.count_nonzero(C-np.diag(np.diag(C)))>0
counts=collections.Counter();changes={m:collections.Counter() for m in MODES[1:]};matched=set();previous=-1
outputs={m:(H/(m+'_events.csv')).open('x') for m in MODES[1:]}
try:
    with (R/'events.csv').open() as f:
        for line in f:
            row=line.rstrip('\n').split(',');counts[row[0]]+=1;assert int(row[1])>=previous;previous=int(row[1]);x=None
            if row[0]=='B':
                assert len(row)==52;counts[row[2]]+=1;key=(row[2],row[3],row[6],row[7]);x=anchors.get(key)
                if x:
                    assert list(map(int,row[8:10]))==[x['start'],x['end']]
                    if int(row[5])==x['revision']:
                        assert int(row[4])==x['ordinal'] and int(row[1])==x['arrival']
                        assert np.array_equal(np.array(row[16:],float),np.array(x['covariance'],float))
                        assert np.max(abs(np.array(row[10:16],float)-np.array(x['mean'],float)))<1e-12
                        x['input_mean']=row[10:16];x['input_covariance']=row[16:];matched.add(key)
            late=bool(x and int(row[1])-x['end']>2_500_000_000)
            if late:assert key in matched and int(row[5])>x['revision']
            for mode,out in outputs.items():
                new=row.copy()
                if late:
                    changes[mode]['eligible_rows']+=1
                    if mode in ('hold_mean','hold_both'):new[10:16]=x['input_mean']
                    if mode in ('hold_cov','hold_both'):new[16:]=x['input_covariance']
                replacement=','.join(new)+'\n' if late else line
                if replacement!=line:changes[mode]['changed_rows']+=1
                out.write(replacement)
finally:
    for f in outputs.values():f.close()
assert matched==anchors.keys();assert counts['B']==213750 and counts['I']==35502 and counts['E']==1
for mode in MODES[1:]:
    with (R/'events.csv').open() as f,(H/(mode+'_events.csv')).open() as v:
        for orig,new in zip(f,v,strict=True):
            if orig==new:continue
            a,b=orig.rstrip('\n').split(','),new.rstrip('\n').split(',');assert a[:10]==b[:10] and a[2]=='kimera_front'
            key=(a[2],a[3],a[6],a[7]);x=anchors[key];assert int(a[1])-x['end']>2_500_000_000
            assert b[10:16]==(x['input_mean'] if mode in ('hold_mean','hold_both') else a[10:16])
            assert b[16:]==(x['input_covariance'] if mode in ('hold_cov','hold_both') else a[16:])
oldtree=tree(R/'receiver/archive');save('original_archive_sha256.json',oldtree)
save('anchors.json',{'|'.join(k):v for k,v in anchors.items()});save('input_sha256.json',pins)
save('prepared.json',dict(passed=True,at=now(),events=249253,counts=dict(counts),target_IDs=len(anchors),changes={k:dict(v) for k,v in changes.items()},windows_s=WINDOWS,cutoff_age_s=2.5,original_archive_files=len(oldtree),original_archive_digest=tree_digest(oldtree),variant_sha256={m:sha(H/(m+'_events.csv')) for m in MODES[1:]},protected_files=8873,free_bytes=shutil.disk_usage(H).free,source_runs=0,baseline_required=True))
print(json.dumps(read(H/'prepared.json'),indent=2))
