"""CP-C7 exact-manifest execution, four workers, no broad/recursive deletion.

Reuses the reviewed predicates and canonical-parent-FD unlink safeguards.
All roots pass read-only preflight BEFORE any worker can delete a file.
"""
import collections
import concurrent.futures as cf
import gzip
import json
import multiprocessing
import os
from pathlib import Path
import shutil
import stat
import subprocess
import sys
import cleanup as c

H=c.H
S=H/'execution_shards'
ROOTS=json.loads((H/'scope.json').read_text())['roots']
PLAN=json.loads((H/'plan.json').read_text())
PROTECTED=set(json.loads((H/'protected_paths.json').read_text()))
ACTIVE=[i for i,r in enumerate(ROOTS) if PLAN['by_root'][r].get('files',0)]

def key(i,suffix):return S/f'{i:03d}_{suffix}.jsonl.gz'
def read(i,suffix):
    with gzip.open(key(i,suffix),'rt') as f:
        for line in f:yield json.loads(line)
def root_of(p):
    for b in c.BASES:
        prefix=str(b)+'/'
        if p.startswith(prefix):return prefix+p[len(prefix):].split('/',1)[0]
    raise AssertionError(p)
def make_shards():
    assert not S.exists() and not (H/'deletion_started.json').exists()
    for n,h in PLAN['manifest_hashes'].items():assert c.sha(H/n)==h,n
    S.mkdir();indices={r:i for i,r in enumerate(ROOTS)};counts=collections.Counter()
    for source,suffix in [('targets.jsonl.gz','targets'),('directories.jsonl.gz','directories')]:
        output=None;previous=None
        try:
            for row in c.records(source):
                root=row.get('root') or root_of(row['p']);i=indices[root]
                if i not in ACTIVE:continue
                if i!=previous:
                    if output is not None:output.close()
                    output=gzip.open(key(i,suffix),'xt',compresslevel=1);previous=i
                c.put(output,row)
                if suffix=='targets':counts[root]+=1
        finally:
            if output is not None:output.close()
    assert all(counts[r]==PLAN['by_root'][r].get('files',0) for r in ROOTS)
    # Shards are lossless root partitions; pin their bytes before workers run.
    c.save('shards.json',{'created':c.now(),'target_count':sum(counts.values()),'active_roots':len(ACTIVE),'hashes':{str(p):c.sha(p) for p in sorted(S.iterdir())}})
    print('SHARDS READY',sum(counts.values()),len(ACTIVE),flush=True)

def worker(task):
    i,phase=task;root=Path(ROOTS[i]);count=allocated=dirs_removed=0
    assert root.parent in c.BASES and root.resolve()==root
    if phase=='check':
        for row in read(i,'targets'):
            p=Path(row['p']);assert row['root']==str(root) and p.is_relative_to(root) and p!=root
            assert c.same(p,row['m']),('target changed',p)
            assert c.eligible(p,root,row['m'],PROTECTED)==row['reason'],p
            count+=1;allocated+=row['m'][6]*512
    elif phase=='delete':
        assert (H/'deletion_started.json').exists()
        with gzip.open(key(i,'deleted'),'xt',compresslevel=1) as log,c.ParentFD() as check:
            for row in read(i,'targets'):
                p=Path(row['p']);check.verify(p,row['m'])
                os.unlink(p.name,dir_fd=check.fd)
                c.put(log,{'p':str(p),'bytes':row['m'][6]*512})
                count+=1;allocated+=row['m'][6]*512
        directories=sorted(read(i,'directories'),key=lambda r:r['p'].count('/'),reverse=True)
        with c.ParentFD() as check:
            for row in directories:
                p=Path(row['p'])
                if p==root:continue
                assert p.is_relative_to(root)
                check.bind(p.parent)
                m=c.statmeta(os.stat(p.name,dir_fd=check.fd,follow_symlinks=False))
                assert stat.S_ISDIR(m[2]) and m[:3]==row['m'][:3],p
                try:os.rmdir(p.name,dir_fd=check.fd);dirs_removed+=1
                except OSError as e:
                    if e.errno not in (39,17):raise
        for row in read(i,'targets'):assert not os.path.lexists(row['p']),row['p']
    else:raise ValueError(phase)
    assert count==PLAN['by_root'][str(root)]['files']
    assert allocated==PLAN['by_root'][str(root)]['bytes']
    result={'root':str(root),'phase':phase,'passed':True,'files':count,'file_allocated_bytes':allocated,'deleted_empty_directories':dirs_removed,'at':c.now()}
    with (S/f'{i:03d}_{phase}_receipt.json').open('x') as f:json.dump(result,f,indent=2)
    return result

def pool_phase(phase):
    results=[]
    with cf.ProcessPoolExecutor(max_workers=4,mp_context=multiprocessing.get_context('fork')) as pool:
        futures=[pool.submit(worker,(i,phase)) for i in ACTIVE]
        try:
            for future in cf.as_completed(futures):
                row=future.result();results.append(row)
                print(phase,len(results),'/',len(ACTIVE),Path(row['root']).name,row['files'],flush=True)
        except BaseException:
            for f in futures:f.cancel()
            raise
    c.save(f'{phase}_receipts.json',results)
    return results

def execute():
    assert not (H/'deletion_started.json').exists(),'No automatic retry after removal begins'
    for n,h in PLAN['manifest_hashes'].items():assert c.sha(H/n)==h,n
    shard_manifest=json.loads((H/'shards.json').read_text())
    for p,h in shard_manifest['hashes'].items():assert c.sha(p)==h,p
    pool_phase('check')
    c.verify_retained();c.check_pins(json.loads((H/'pins.json').read_text()))
    subprocess.run(['python3','-B',str(H/'host_audit.py'),'admission'],check=True)
    before=shutil.disk_usage(c.W).free
    c.save('deletion_started.json',{'at':c.now(),'free_bytes':before,'plan_sha256':c.sha(H/'plan.json'),'shards_sha256':c.sha(H/'shards.json'),'cleanup_sha256':c.sha(H/'cleanup.py'),'executor_sha256':c.sha(H/'parallel_execute.py'),'host_audit_sha256':c.sha(H/'host_audit.py'),'workers':4})
    results=pool_phase('delete')
    assert sum(r['files'] for r in results)==PLAN['target_files']
    kept=c.verify_retained();pins=c.check_pins(json.loads((H/'pins.json').read_text()))
    cp=subprocess.run(['python3','-B',str(c.CP/'verify_checkpoint.py'),'--originals'],capture_output=True,text=True,check=True).stdout.strip()
    after=shutil.disk_usage(c.W).free
    c.save('completion.json',{'at':c.now(),'passed':True,'deleted_files':sum(r['files'] for r in results),'deleted_empty_directories':sum(r['deleted_empty_directories'] for r in results),'file_allocated_bytes':sum(r['file_allocated_bytes'] for r in results),'affected_roots':len(results),'free_before':before,'free_after':after,'free_change':after-before,'retained_entries_verified':kept,'pins_verified':pins,'dataset_files_verified':len(json.loads((H/'datasets_before.json').read_text())),'hesai_checkpoint':cp,'recoverable_from_trash':False,'backup_of_deleted_payloads':False,'workers':4})
    print('COMPLETE',round((after-before)/2**30,3),'GiB freed;',round(after/2**30,3),'GiB free',flush=True)

if __name__=='__main__':{'shard':make_shards,'execute':execute}[sys.argv[1]]()
