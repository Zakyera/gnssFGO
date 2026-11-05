"""CP-C7 finite cleanup. Each phase is explicit; unknown content is retained."""
from pathlib import Path
import collections
import datetime
import gzip
import hashlib
import json
import os
import re
import shutil
import stat
import subprocess
import sys

H = Path(__file__).resolve().parent
R = Path('/home/yeranis/repos/gnssFGO')
W = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
BASES = [W/'runs', Path('/home/yeranis/results')]
CP = R/'experiments/checkpoints/20261023_hesai_four_sequences'
PIN = W/'runs/eth1_split_imu_persistent_on_cpv91_20261102_attempt1/prepared/protected_manifest.json'
DATA = W/'src/datasets'
RETAIN_DIR = re.compile(r'(^|_)(build|runtime|src|include|lib|bin|test|tests|fixture|fixtures|sanitizer|sanitizers|checkpoint|checkpoints|config|configs|profile|profiles|calibration|dataset|datasets|input|inputs)(_|$)|^(source|sources)$',re.I)
CODE_SUFFIX = {'.py','.sh','.cpp','.cc','.c','.h','.hpp','.inc','.o','.a','.so','.d','.cmake','.yaml','.yml','.launch','.xml','.toml','.md','.json','.tum','.png','.pdf','.svg','.zip','.gz','.npz'}
SNAP_FILES = set('records.txt summary.csv incremental_states.csv incremental_errors.csv ledger.csv batch_errors.csv batch_states.csv factor_payload_probe.txt full_backsub_errors.csv full_backsub_linear_health.csv full_backsub_states.csv incremental_dense_poses.csv incremental_linear_health.csv iterations.csv polished_dense_poses.csv polished_errors.csv polished_linear_health.csv polished_states.csv profile_validation.csv source_residuals.csv source_weights.csv'.split())
RAW_NAMES = set('estimators.log e1b_profile.jsonl stream_progress.csv events.csv receipts.jsonl source_revision_events.csv revision_events.csv'.split())

def now(): return datetime.datetime.now().astimezone().isoformat()
def save(name,obj):
    with (H/name).open('x') as f: json.dump(obj,f,indent=2)
def sha(p):
    with open(p,'rb') as f: return hashlib.file_digest(f,'sha256').hexdigest()
def meta(p):
    s=os.lstat(p)
    return statmeta(s)
def statmeta(s):
    return [s.st_dev,s.st_ino,s.st_mode,s.st_nlink,s.st_size,s.st_mtime_ns,s.st_blocks]
def same(p,m): return meta(p)==m
def records(name):
    with gzip.open(H/name,'rt') as f:
        for line in f: yield json.loads(line)
def put(f,obj): f.write(json.dumps(obj,separators=(',',':'))+'\n')
def old_name(name):
    # Fix-development fixtures are intentionally retained, even pre-fix ones.
    v=re.search(r'cpv(\d+)',name)
    if v and int(v[1])>=35: return False
    if name.startswith('rerun_method') and name.endswith('_all_sequences_20261029'): return True
    date=re.search(r'(2026\d{4})(?:_attempt\d+)?$',name)
    if date: return date[1]<'20261029'
    return name.startswith('grandtour_') or name=='granttour'
def scope():
    rows=[]; keep=[]
    for b in BASES:
        for p in sorted(b.iterdir()):
            if not p.is_dir() or p.is_symlink(): continue
            if old_name(p.name): rows.append(str(p))
            else: keep.append(str(p))
    save('scope.json',dict(created=now(),roots=rows,excluded_roots=keep,boundary='CP-V38/39/40; retain CP-V35+ and all undated unknowns'))
    print('Reviewed-scope candidates',len(rows),'retained roots',len(keep),flush=True)

def strings(o):
    if isinstance(o,str): yield o
    elif isinstance(o,dict):
        for k,v in o.items(): yield from strings(k); yield from strings(v)
    elif isinstance(o,list):
        for x in o: yield from strings(x)
def protections():
    pins=json.loads(PIN.read_text()); protected=set(pins)
    cp=json.loads((CP/'manifest.json').read_text())
    for name,v in cp['files'].items():
        pins[str(CP/name)]=v['sha256']; pins[v['original']]=v['sha256']
    # Preserve post-fix result/input references as well as the latest manifest.
    sources=[]
    for exp in (R/'experiments').iterdir():
        v=re.search(r'cpv(\d+)',exp.name)
        if exp.is_dir() and v and int(v[1])>=35 and exp!=H:
            sources.extend(exp.glob('*.json'))
    for base in BASES:
        for root in base.iterdir():
            v=re.search(r'cpv(\d+)',root.name)
            if root.is_dir() and v and int(v[1])>=35:
                sources.extend(root.glob('prepared/*manifest*.json'))
                sources.extend(root.glob('*manifest*.json'))
    for f in sources:
        if f.stat().st_size>30*1024**2: continue
        try: obj=json.loads(f.read_text())
        except (ValueError,UnicodeDecodeError): continue
        for s in strings(obj):
            if s.startswith('/home/yeranis/') and len(s)<4096 and '\n' not in s:
                try:
                    p=Path(s)
                    if p.is_file(): protected.add(s);protected.add(str(p.resolve()))
                except (OSError,ValueError): pass
    protected.update(pins)
    protected.update(str(Path(p).resolve()) for p in list(protected))
    return pins,protected
def check_pins(pins):
    for i,(p,h) in enumerate(pins.items()):
        assert sha(p)==h,('protected content mismatch',p)
        if i and i%2000==0: print('pins verified',i,flush=True)
    return len(pins)
def dataset_meta():
    out={}
    for parent,dirs,names in os.walk(DATA,followlinks=False):
        for n in names:
            p=Path(parent)/n
            out[str(p)]=meta(p)
    return out
def eligible(p,root,m,protected):
    if str(p) in protected or m[3]!=1 or not stat.S_ISREG(m[2]): return None
    parts=p.relative_to(root).parts
    if any(RETAIN_DIR.search(x) for x in parts[:-1]): return None
    snapshot=any(re.fullmatch(r'snapshot_\d+',x) or (x.isdigit() and i and parts[i-1]=='commits') for i,x in enumerate(parts[:-1]))
    if snapshot and p.name in SNAP_FILES: return 'intermediate_receiver_snapshot'
    if p.suffix=='.rrd': return 'obsolete_visualization'
    if p.suffix=='.bag' and 'recordings' in parts: return 'generated_recording'
    if 'glim_dump' in parts and p.suffix in {'.bin','.ply','.pcd'}: return 'generated_map'
    if p.suffix in CODE_SUFFIX or '.so.' in p.name or m[2]&0o111: return None
    if m[4]>=1024**2 and (p.name in RAW_NAMES or p.suffix in {'.log','.jsonl'}): return 'large_raw_diagnostic'
    # Everything else, including compact trajectories/tables and unknowns, stays.
    return None
def plan():
    assert not (H/'plan.json').exists() and not (H/'deletion_started.json').exists()
    s=json.loads((H/'scope.json').read_text()); pins,protected=protections()
    check_pins(pins)
    save('pins.json',pins);save('protected_paths.json',sorted(protected))
    save('datasets_before.json',dataset_meta())
    totals=collections.Counter(); byroot={}; skipped=[]
    with gzip.open(H/'targets.jsonl.gz','xt',compresslevel=1) as targets, gzip.open(H/'retained.jsonl.gz','xt',compresslevel=1) as retained, gzip.open(H/'directories.jsonl.gz','xt',compresslevel=1) as directories:
        for rootname in s['roots']:
            root=Path(rootname);assert root.resolve()==root and root.parent in BASES
            counts=collections.Counter()
            for parent,dirs,names in os.walk(root,followlinks=False):
                parent=Path(parent)
                put(directories,{'p':str(parent),'m':meta(parent)})
                # Do not traverse build/runtime/fixture/profile trees at all.
                for d in list(dirs):
                    q=parent/d
                    if q.is_symlink() or RETAIN_DIR.search(d):
                        dirs.remove(d);skipped.append(str(q))
                for n in names:
                    p=parent/n;m=meta(p);reason=eligible(p,root,m,protected)
                    row={'p':str(p),'m':m,'root':rootname}
                    if reason:
                        # Root canonicality plus non-following walk/regular lstat;
                        # canonical parent FD is checked again before deletion.
                        row['reason']=reason;put(targets,row)
                        counts['files']+=1;counts['bytes']+=m[6]*512
                        totals[reason]+=1
                    else:
                        if stat.S_ISLNK(m[2]): row['link']=os.readlink(p)
                        put(retained,row)
                        counts['retained']+=1
            byroot[rootname]=dict(counts)
            print('inventoried',root.name,dict(counts),flush=True)
    save('skipped_trees.json',skipped)
    result=dict(created=now(),by_root=byroot,reasons=totals,target_files=sum(x.get('files',0) for x in byroot.values()),target_allocated_bytes=sum(x.get('bytes',0) for x in byroot.values()),retained_files=sum(x.get('retained',0) for x in byroot.values()),pins=len(pins),protected_paths=len(protected),free_bytes=shutil.disk_usage(W).free)
    result['manifest_hashes']={n:sha(H/n) for n in ['scope.json','targets.jsonl.gz','retained.jsonl.gz','directories.jsonl.gz','pins.json','protected_paths.json','datasets_before.json','skipped_trees.json']}
    save('plan.json',result)
    print('PLAN',result['target_files'],result['target_allocated_bytes']/2**30,'GiB',flush=True)
def verify_retained():
    count=0
    for row in records('retained.jsonl.gz'):
        assert same(row['p'],row['m']),('retained identity changed',row['p'])
        if 'link' in row: assert os.readlink(row['p'])==row['link']
        count+=1
    assert dataset_meta()==json.loads((H/'datasets_before.json').read_text()),'dataset identities changed'
    return count
def execute():
    assert not (H/'deletion_started.json').exists(),'No blind repeat after destructive phase begins'
    plan=json.loads((H/'plan.json').read_text())
    for n,h in plan['manifest_hashes'].items(): assert sha(H/n)==h,n
    roots=set(json.loads((H/'scope.json').read_text())['roots']); protected=set(json.loads((H/'protected_paths.json').read_text()))
    # Host safety receipt is generated and inspected separately immediately before launch.
    host=json.loads((H/'host_safety.json').read_text())
    assert host['passed'] and datetime.datetime.now().timestamp()-host['epoch']<300
    for row in records('targets.jsonl.gz'):
        p=Path(row['p']);root=Path(row['root'])
        assert str(root) in roots and root.parent in BASES and p.is_relative_to(root) and p!=root
        assert same(p,row['m']),('target changed',p)
        assert eligible(p,root,row['m'],protected)==row['reason'],p
    verify_retained(); check_pins(json.loads((H/'pins.json').read_text()))
    # Recheck host state after the potentially lengthy all-target preflight.
    subprocess.run(['python3','-B',str(H/'host_audit.py'),'admission'],check=True)
    before=shutil.disk_usage(W).free
    save('deletion_started.json',dict(at=now(),free_bytes=before,plan_sha256=sha(H/'plan.json'),cleanup_sha256=sha(H/'cleanup.py'),host_audit_sha256=sha(H/'host_audit.py')))
    count=allocated=0
    with gzip.open(H/'deleted.jsonl.gz','xt',compresslevel=1) as log, ParentFD() as check:
        for row in records('targets.jsonl.gz'):
            p=Path(row['p']);check.verify(p,row['m'])
            os.unlink(p.name,dir_fd=check.fd);put(log,{'p':str(p),'bytes':row['m'][6]*512})
            count+=1;allocated+=row['m'][6]*512
            if count%50000==0: print('deleted files',count,'allocatedGiB',round(allocated/2**30,3),flush=True)
    dirs=list(records('directories.jsonl.gz'));removed_dirs=0
    with ParentFD() as check:
        for row in sorted(dirs,key=lambda x:x['p'].count('/'),reverse=True):
            p=Path(row['p'])
            if str(p) in roots: continue
            check.bind(p.parent)
            m=statmeta(os.stat(p.name,dir_fd=check.fd,follow_symlinks=False))
            assert stat.S_ISDIR(m[2]) and m[:3]==row['m'][:3],p
            try:os.rmdir(p.name,dir_fd=check.fd);removed_dirs+=1
            except OSError as e:
                if e.errno not in (39,17): raise
    for row in records('targets.jsonl.gz'): assert not os.path.lexists(row['p'])
    kept=verify_retained();npins=check_pins(json.loads((H/'pins.json').read_text()))
    cp=subprocess.run(['python3','-B',str(CP/'verify_checkpoint.py'),'--originals'],capture_output=True,text=True,check=True).stdout.strip()
    after=shutil.disk_usage(W).free
    save('completion.json',dict(at=now(),passed=True,deleted_files=count,deleted_empty_directories=removed_dirs,file_allocated_bytes=allocated,free_before=before,free_after=after,free_change=after-before,retained_entries_verified=kept,pins_verified=npins,dataset_files_verified=len(json.loads((H/'datasets_before.json').read_text())),hesai_checkpoint=cp,recoverable_from_trash=False,backup_of_deleted_payloads=False))
    print('COMPLETE',count,'files',round((after-before)/2**30,3),'GiB freed',round(after/2**30,3),'GiB free',flush=True)

class ParentFD:
    """Pin exact canonical directory while checking/unlinking regular files."""
    def __init__(self): self.fd=None;self.parent=None
    def __enter__(self):return self
    def __exit__(self,*args):
        if self.fd is not None:os.close(self.fd)
    def bind(self,parent):
        if parent!=self.parent:
            if self.fd is not None:os.close(self.fd)
            self.fd=os.open(parent,os.O_RDONLY|os.O_DIRECTORY|os.O_NOFOLLOW)
            self.parent=parent
            assert os.readlink(f'/proc/self/fd/{self.fd}')==str(self.parent),('noncanonical parent',parent)
    def verify(self,p,expected):
        self.bind(p.parent)
        actual=statmeta(os.stat(p.name,dir_fd=self.fd,follow_symlinks=False))
        assert actual==expected and stat.S_ISREG(actual[2]) and actual[3]==1,('target changed',p)

if __name__=='__main__':
    {'scope':scope,'plan':plan,'execute':execute}[sys.argv[1]]()
