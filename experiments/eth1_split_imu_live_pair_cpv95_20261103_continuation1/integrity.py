"""Read-only preservation of frozen files and the complete saved OFF trees."""
import hashlib,json,os,stat
from pathlib import Path
from paths import H,P
def sha(path):
    with Path(path).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def tree_snapshot(root):
    root=Path(root);h=hashlib.sha256();files=dirs=links=fifos=total=0
    for folder,names,leaves in os.walk(root,followlinks=False):
        names.sort();leaves.sort()
        for name in names+leaves:
            p=Path(folder)/name;s=p.lstat();relative=str(p.relative_to(root))
            if stat.S_ISLNK(s.st_mode):kind='link';value=os.readlink(p);links+=1
            elif stat.S_ISDIR(s.st_mode):kind='directory';value=None;dirs+=1
            elif stat.S_ISFIFO(s.st_mode):
                assert root==P/'full_off_receiver' and relative=='events.fifo',p
                # The stopped receiver's named pipe has no file payload. Never
                # open/read it (that would block); preserve its type and mode.
                kind='fifo';value=None;fifos+=1
            else:
                assert stat.S_ISREG(s.st_mode),p
                kind='file';value=sha(p);files+=1;total+=s.st_size
            h.update((json.dumps([relative,kind,stat.S_IMODE(s.st_mode),value],separators=(',',':'))+'\n').encode())
    return dict(root=str(root),tree_sha256=h.hexdigest(),files=files,directories=dirs,symlinks=links,fifos=fifos,bytes=total)
def verify_off():
    saved=json.loads((H/'preserved_off.json').read_text())
    for name,expected in saved.items():
        actual=tree_snapshot(P/name)
        assert actual==expected,('Saved OFF tree changed',name,actual,expected)
    return {name:v['files'] for name,v in saved.items()}
