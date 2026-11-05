"""Reconciled CP-C7 continuation after ownership failure; no privilege workaround."""
from pathlib import Path
import concurrent.futures as cf
import gzip,hashlib,importlib.util,json,multiprocessing,os,shutil,stat,subprocess,sys
H=Path(__file__).resolve().parent;ORIG=H.parent
spec=importlib.util.spec_from_file_location('cpc7_cleanup',ORIG/'cleanup.py');c=importlib.util.module_from_spec(spec);spec.loader.exec_module(c)
PLAN=json.loads((ORIG/'plan.json').read_text());ROOTS=json.loads((ORIG/'scope.json').read_text())['roots']
ACTIVE=[i for i,r in enumerate(ROOTS) if PLAN['by_root'][r].get('files',0)]
S=H/'shards'
def path(i,kind,old=False):return (ORIG/'execution_shards' if old else S)/f'{i:03d}_{kind}.jsonl.gz'
def read(p):
    with gzip.open(p,'rt') as f:
        for line in f:yield json.loads(line)
def save(p,d):
    with p.open('x') as f:json.dump(d,f,indent=2)
def scan(i):
    root=Path(ROOTS[i]);assert root.parent in c.BASES and root.resolve()==root
    logged={}
    if path(i,'deleted',True).exists():
        for row in read(path(i,'deleted',True)):
            assert row['p'] not in logged;logged[row['p']]=row['bytes']
    prior=remaining=blocked=prior_bytes=remaining_bytes=blocked_bytes=0;access={}
    with gzip.open(path(i,'targets'),'xt',compresslevel=1) as yes,gzip.open(path(i,'blocked'),'xt',compresslevel=1) as no:
        for row in read(path(i,'targets',True)):
            p=Path(row['p']);assert row['root']==str(root) and p.is_relative_to(root) and p!=root
            if not os.path.lexists(p):
                assert str(p) in logged and logged[str(p)]==row['m'][6]*512,('unexplained missing target',p)
                prior+=1;prior_bytes+=row['m'][6]*512;continue
            assert str(p) not in logged and c.same(p,row['m']),('changed/recreated target',p)
            parent=str(p.parent)
            if parent not in access:access[parent]=os.access(parent,os.W_OK|os.X_OK)
            if access[parent]:c.put(yes,row);remaining+=1;remaining_bytes+=row['m'][6]*512
            else:c.put(no,row);blocked+=1;blocked_bytes+=row['m'][6]*512
    assert prior==len(logged)
    assert prior+remaining+blocked==PLAN['by_root'][str(root)]['files']
    receipt=ORIG/'execution_shards'/f'{i:03d}_delete_receipt.json'
    prior_dirs=json.loads(receipt.read_text())['deleted_empty_directories'] if receipt.exists() else 0
    with gzip.open(path(i,'directories'),'xt',compresslevel=1) as out:
        for row in read(path(i,'directories',True)):
            p=Path(row['p'])
            if not p.exists():assert receipt.exists(),('unexplained missing directory',p)
            else:assert c.meta(p)[:3]==row['m'][:3];c.put(out,row)
    result={'root':str(root),'i':i,'prior_files':prior,'prior_bytes':prior_bytes,'remaining_files':remaining,'remaining_bytes':remaining_bytes,'blocked_files':blocked,'blocked_bytes':blocked_bytes,'prior_empty_directories':prior_dirs}
    save(S/f'{i:03d}_reconciliation.json',result);return result
def prepare():
    assert not S.exists() and not (H/'started.json').exists()
    for n,h in PLAN['manifest_hashes'].items():assert c.sha(ORIG/n)==h,n
    S.mkdir();results=[]
    with cf.ProcessPoolExecutor(max_workers=4,mp_context=multiprocessing.get_context('fork')) as pool:
        for row in pool.map(scan,ACTIVE):
            results.append(row);print('reconciled',len(results),'/',len(ACTIVE),Path(row['root']).name,row['blocked_files'],'blocked',flush=True)
    c.verify_retained();c.check_pins(json.loads((ORIG/'pins.json').read_text()))
    result={'at':c.now(),'roots':results,'prior_files':sum(r['prior_files'] for r in results),'remaining_files':sum(r['remaining_files'] for r in results),'blocked_files':sum(r['blocked_files'] for r in results),'blocked_bytes':sum(r['blocked_bytes'] for r in results),'hashes':{str(p):c.sha(p) for p in S.iterdir() if p.suffix=='.gz'}}
    assert result['prior_files']+result['remaining_files']+result['blocked_files']==PLAN['target_files']
    save(H/'prepared.json',result);print('READY', {k:v for k,v in result.items() if k not in ('roots','hashes')},flush=True)
def delete_one(row):
    i=row['i'];root=Path(row['root']);n=b=removed_dirs=blocked_dirs=0
    assert (H/'started.json').exists()
    with gzip.open(path(i,'deleted'),'xt',compresslevel=1) as log,c.ParentFD() as check:
        for target in read(path(i,'targets')):
            p=Path(target['p']);assert p.is_relative_to(root) and p!=root
            check.verify(p,target['m']);os.unlink(p.name,dir_fd=check.fd)
            c.put(log,{'p':str(p),'bytes':target['m'][6]*512});n+=1;b+=target['m'][6]*512
    with c.ParentFD() as check:
        for d in sorted(read(path(i,'directories')),key=lambda x:x['p'].count('/'),reverse=True):
            p=Path(d['p'])
            if p==root:continue
            assert p.is_relative_to(root)
            if not os.access(p.parent,os.W_OK|os.X_OK):blocked_dirs+=1;continue
            check.bind(p.parent);m=c.statmeta(os.stat(p.name,dir_fd=check.fd,follow_symlinks=False))
            assert stat.S_ISDIR(m[2]) and m[:3]==d['m'][:3]
            try:os.rmdir(p.name,dir_fd=check.fd);removed_dirs+=1
            except OSError as e:
                if e.errno not in (39,17):raise
    for t in read(path(i,'targets')):assert not os.path.lexists(t['p'])
    for t in read(path(i,'blocked')):assert c.same(t['p'],t['m'])
    assert n==row['remaining_files'] and b==row['remaining_bytes']
    result={'i':i,'root':str(root),'deleted_files':n,'file_allocated_bytes':b,'deleted_empty_directories':removed_dirs,'permission_blocked_directories':blocked_dirs,'passed':True}
    save(S/f'{i:03d}_completion.json',result);return result
def execute():
    assert not (H/'started.json').exists() and not (ORIG/'completion.json').exists()
    prep=json.loads((H/'prepared.json').read_text())
    for p,h in prep['hashes'].items():assert c.sha(p)==h
    c.verify_retained();c.check_pins(json.loads((ORIG/'pins.json').read_text()))
    subprocess.run(['python3','-B',str(H/'host_audit.py'),'admission'],check=True)
    save(H/'started.json',{'at':c.now(),'free_bytes':shutil.disk_usage(c.W).free,'prepared_sha256':c.sha(H/'prepared.json'),'script_sha256':c.sha(__file__),'workers':4})
    results=[];selected=[r for r in prep['roots'] if r['remaining_files']]
    with cf.ProcessPoolExecutor(max_workers=4,mp_context=multiprocessing.get_context('fork')) as pool:
        futures=[pool.submit(delete_one,r) for r in selected]
        try:
            for f in cf.as_completed(futures):
                row=f.result();results.append(row);print('deleted',len(results),'/',len(selected),Path(row['root']).name,row['deleted_files'],flush=True)
        except BaseException:
            for f in futures:f.cancel()
            raise
    for r in prep['roots']:
        for t in read(path(r['i'],'blocked')):assert c.same(t['p'],t['m'])
    retained=c.verify_retained();pins=c.check_pins(json.loads((ORIG/'pins.json').read_text()))
    cp=subprocess.run(['python3','-B',str(c.CP/'verify_checkpoint.py'),'--originals'],text=True,capture_output=True,check=True).stdout.strip()
    prior=json.loads((ORIG/'deletion_started.json').read_text());free=shutil.disk_usage(c.W).free
    total=prep['prior_files']+sum(r['deleted_files'] for r in results)
    assert total+prep['blocked_files']==PLAN['target_files']
    result={'at':c.now(),'passed':True,'all_planned_files_deleted':prep['blocked_files']==0,'deleted_files':total,'deleted_empty_directories':sum(r['prior_empty_directories'] for r in prep['roots'])+sum(r['deleted_empty_directories'] for r in results),'file_allocated_bytes':sum(r['prior_bytes'] for r in prep['roots'])+sum(r['file_allocated_bytes'] for r in results),'permission_blocked_files':prep['blocked_files'],'permission_blocked_bytes':prep['blocked_bytes'],'affected_roots':len([r for r in prep['roots'] if r['prior_files'] or r['remaining_files']]),'free_before':prior['free_bytes'],'free_after':free,'free_change':free-prior['free_bytes'],'retained_entries_verified':retained,'pins_verified':pins,'dataset_files_verified':len(json.loads((ORIG/'datasets_before.json').read_text())),'hesai_checkpoint':cp,'recoverable_from_trash':False,'backup_of_deleted_payloads':False,'ownership_or_permissions_changed':False,'reconciled_continuation':str(H)}
    save(H/'completion.json',result);save(ORIG/'completion.json',result)
    print('COMPLETE',result,flush=True)
if __name__=='__main__':{'prepare':prepare,'execute':execute}[sys.argv[1]]()
