"""Temporarily relocate only six new CP-V28 NPZs to verified RAM copies."""
import hashlib,json,shutil,sys,tempfile
from pathlib import Path
H=Path(__file__).resolve().parent
def sha(p):
    with p.open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
if sys.argv[1]=='stage':
    dest=Path(tempfile.mkdtemp(prefix='cpv28-products-',dir='/run/user/1000'))
    paths=sorted(H.glob('*_glim_products.npz'));assert len(paths)==6
    receipt=[]
    for path in paths:
        assert not path.is_symlink()
        target=dest/path.name;digest=sha(path)
        shutil.copy2(path,target);assert sha(target)==digest
        receipt.append(dict(path=str(path),temporary=str(target),sha256=digest))
    with (H/'staged_products.json').open('x') as f:json.dump(receipt,f,indent=2)
    for row in receipt:
        path=Path(row['path']);assert sha(path)==row['sha256']
        path.unlink();path.symlink_to(row['temporary'])
    print(dest)
elif sys.argv[1]=='restore':
    rows=json.loads((H/'staged_products.json').read_text())
    for row in rows:
        path,temp=Path(row['path']),Path(row['temporary'])
        assert path.is_symlink() and path.resolve()==temp and sha(temp)==row['sha256']
        path.unlink();shutil.copy2(temp,path);assert sha(path)==row['sha256'];temp.unlink()
    parent=Path(rows[0]['temporary']).parent
    assert parent.parent==Path('/run/user/1000') and parent.name.startswith('cpv28-products-')
    parent.rmdir()
    with (H/'staged_products_restored.json').open('x') as f:json.dump(dict(pass_=True,files=len(rows)),f)
else:raise ValueError(sys.argv[1])
