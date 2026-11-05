"""Explicit-manifest deletion of obsolete non-GrandTour outputs, not datasets/code."""
from pathlib import Path
import argparse, collections, datetime, hashlib, json, os, re, stat, subprocess

H = Path(__file__).resolve().parent
W = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
V = W.parent
E = Path('/home/yeranis/repos/gnssFGO/experiments')
CODE = {'.py', '.sh', '.cpp', '.cc', '.c', '.h', '.hpp', '.patch', '.diff', '.cmake'}
DATA = {'.bag', '.db3', '.mcap'}
STAGES = ['stage2a_implementation', 'stage2b_implementation', 'stage2c_b',
          'stage2c_c', 'stage2c_d1', 'stage2c_d2', 'stage2c_medium',
          'stage3a_characterization', 'stage15_diagnosis', 'stage16_diagnosis',
          'stage17_diagnosis', 'stage18_diagnosis']

def now(): return datetime.datetime.now().astimezone().isoformat()
def digest(p):
    with Path(p).open('rb') as f: return hashlib.file_digest(f, 'sha256').hexdigest()
def save(name, obj):
    with (H/name).open('x') as f: json.dump(obj, f, indent=2); f.write('\n')
def free():
    s = os.statvfs(W); return s.f_bavail*s.f_frsize
def within(p, r): return p == r or r in p.parents
def identity(p):
    s=p.lstat()
    return dict(dev=s.st_dev, ino=s.st_ino, size=s.st_size, mode=s.st_mode,
                mtime=s.st_mtime_ns, blocks=s.st_blocks*512,
                link=os.readlink(p) if p.is_symlink() else None)

def classify(d):
    a=d.get('launch_args', {})
    relevant=' '.join(str(x) for x in (d.get('ground_truth',''), d.get('launch_file',''),
                                      a.get('bag_path',''), d.get('bag','')))
    if re.search('grandtour|grand_tour|grand tour', relevant, re.I): return None
    for tag in ('S3E','Boreas','M3DGR','GEODE'):
        if tag.lower() in relevant.lower(): return tag

def candidates():
    result=[]; unknown=[]
    for base in (V/'runs', W/'runs', W/'stage2b_implementation/runs'):
        for p in sorted(base.iterdir()):
            if not p.is_dir() or p.is_symlink() or not re.match(r'2026\d{4}-\d{6}_',p.name): continue
            m=p/'manifest.json'; tag=None; evidence=None
            if m.is_file():
                tag=classify(json.loads(m.read_text())); evidence=str(m)
            if not tag and not m.exists():
                for name in ('roslaunch.log','rosbag.log'):
                    q=p/name
                    if not q.is_file(): continue
                    with q.open(errors='replace') as f: prefix=f.read(512*1024)
                    if re.search(r'(?:Opening |bag_path[^\n]*)(?:[^\n]*S3E[^\n]*\.bag)',prefix,re.I):
                        tag='S3E'; evidence=str(q); break
            if tag: result.append(dict(path=str(p), dataset=tag, evidence=evidence))
            else: unknown.append(str(p))
    # These old stage reports/manifest files identify M3DGR or GEODE. Restrict to
    # generated run directories; no stage source/build/config/raw-data tree.
    for stage in STAGES:
        base=W/stage
        for directory,dirs,files in os.walk(base,followlinks=False):
            d=Path(directory)
            dirs[:]=[n for n in dirs if not (d/n).is_symlink() and
                n not in ('src','.git','source','control_source','tools','tests') and
                not n.startswith('safety') and not n.endswith(('_ws','_build','_devel','_logs'))]
            for n in list(dirs):
                if n=='runs' or n.endswith('_runs'):
                    result.append(dict(path=str(d/n),dataset='GEODE' if stage.startswith(('stage2c','stage3a')) else 'M3DGR/legacy checks',
                                       evidence='stage report and run manifests (reviewed)'))
                    dirs.remove(n)
    result.sort(key=lambda x:len(Path(x['path']).parts))
    roots=[]
    for x in result:
        if not any(within(Path(x['path']),Path(r['path'])) for r in roots): roots.append(x)
    return roots,unknown

def protections():
    paths=set()
    old=json.loads((E/'artifact_cleanup_cpc3_20261026/protected_before.json').read_text())
    paths.update(Path(p) for p in old)
    def extract(obj):
        if isinstance(obj,dict):
            for k,v in obj.items():
                if isinstance(k,str) and k.startswith('/home/yeranis/') and isinstance(v,str) and re.fullmatch('[a-f0-9]{64}',v): paths.add(Path(k))
                extract(v)
        elif isinstance(obj,list):
            for v in obj:extract(v)
    for folder in ('newest_absolute_comparison_cpv27_20261028','all_incremental_comparison_cpv28_20261028'):
        for name in ('closeout.json','input_pins.json','replay_input_pins.json'):
            p=E/folder/name
            if p.exists(): extract(json.loads(p.read_text()));paths.add(p)
    return sorted(p for p in paths if p.is_file())

def scan(root):
    root=Path(root)
    assert root.resolve()==root and not root.is_symlink() and not os.path.ismount(root)
    assert root not in (V,W,V/'runs',W/'runs') and within(root,V)
    for directory,dirs,files in os.walk(root,followlinks=False):
        d=Path(directory)
        assert d.stat().st_dev==root.stat().st_dev,d
        links=[n for n in dirs if (d/n).is_symlink()]
        dirs[:]=[n for n in dirs if n not in links]
        for n in sorted(files+links):
            p=d/n
            assert p.parent.resolve()==p.parent,p
            yield p

def prepare():
    roots,unknown=candidates(); pins=protections()
    resolved_pins=[p.resolve() for p in pins]
    for row in roots:
        r=Path(row['path'])
        assert not any(within(p,r) for p in pins+resolved_pins),r
    print('Scope and dependencies checked:',len(roots),'roots',len(pins),'pins',flush=True)
    inventory=[];counts=collections.defaultdict(lambda:dict(delete_bytes=0,delete_files=0,kept_files=0))
    for r in roots:
        for p in scan(r['path']):
            rel=p.relative_to(r['path'])
            keep=(p.suffix in CODE|DATA or any(x in ('.git','src','source_snapshot') or x.startswith('safety') for x in rel.parts))
            row=dict(path=str(p),root=r['path'],dataset=r['dataset'],action='keep' if keep else 'delete',**identity(p))
            assert stat.S_ISREG(row['mode']) or stat.S_ISLNK(row['mode']),p
            if keep and not p.is_symlink():row['sha256']=digest(p)
            inventory.append(row)
            count=counts[r['dataset']]
            if keep:count['kept_files']+=1
            else:count['delete_files']+=1;count['delete_bytes']+=row['blocks']
    save('inventory.json',inventory)
    save('plan.json',dict(time=now(),roots=roots,unknown_retained=unknown,totals=counts,
         free_bytes=free(),inventory_sha256=digest(H/'inventory.json')))
    print(json.dumps(dict(roots=len(roots),totals=counts,unknown=unknown),indent=2),flush=True)
    protected={str(p):dict(sha256=digest(p),resolved=str(p.resolve())) for p in pins}
    save('protected_before.json',protected)
    print('Protected hashes:',len(protected),flush=True)

def apply():
    plan=json.loads((H/'plan.json').read_text()); rows=json.loads((H/'inventory.json').read_text())
    assert digest(H/'inventory.json')==plan['inventory_sha256']
    pins=json.loads((H/'protected_before.json').read_text())
    assert not (H/'deletion_started.json').exists()
    roots=[Path(r['path']) for r in plan['roots']]
    for p,s in pins.items():assert digest(p)==s['sha256'] and str(Path(p).resolve())==s['resolved'],p
    def validate(row):
        p=Path(row['path']);r=Path(row['root'])
        assert r in roots and p!=r and within(p,r) and p.parent.resolve()==p.parent,p
        assert identity(p)=={k:row[k] for k in identity(p)},p
        assert os.access(p.parent,os.W_OK|os.X_OK),p
    for row in rows:validate(row)
    before=free();save('deletion_started.json',dict(time=now(),free_bytes=before))
    removed=0
    with (H/'deleted.jsonl').open('x') as f:
        for row in rows:
            if row['action']!='delete':continue
            validate(row);Path(row['path']).unlink();removed+=1
            f.write(json.dumps(dict(path=row['path'],bytes=row['size'],dataset=row['dataset']))+'\n')
    empty=0
    for root in roots:
        for directory,dirs,files in os.walk(root,topdown=False,followlinks=False):
            p=Path(directory)
            if p.is_symlink():continue
            try:p.rmdir();empty+=1
            except OSError:pass
    for row in rows:
        p=Path(row['path'])
        if row['action']=='delete':assert not os.path.lexists(p),p
        elif 'sha256' in row:assert digest(p)==row['sha256'],p
    for p,s in pins.items():assert digest(p)==s['sha256'] and str(Path(p).resolve())==s['resolved'],p
    check=subprocess.run(['python3',str(E/'checkpoints/20261023_hesai_four_sequences/verify_checkpoint.py'),'--originals'],capture_output=True,text=True)
    assert check.returncode==0,check.stdout+check.stderr
    result=dict(passed=True,time=now(),removed_files=removed,removed_empty_directories=empty,
         free_before=before,free_after=free(),freed_bytes=free()-before,protected_files=len(pins),
         checkpoint=check.stdout.strip(),recoverability='Permanent deletion; not moved to Trash; no new bulk backup.')
    save('completion.json',result);print(json.dumps(result,indent=2),flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('action',choices=['prepare','apply']);a=p.parse_args()
    prepare() if a.action=='prepare' else apply()
