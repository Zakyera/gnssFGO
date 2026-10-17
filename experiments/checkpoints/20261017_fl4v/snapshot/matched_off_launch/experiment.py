from pathlib import Path
import hashlib
ROOT=Path(__file__).resolve().parent
REPO=Path('/home/yeranis/repos/gnssFGO')
def file_sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        for b in iter(lambda:f.read(1048576),b''):h.update(b)
    return h.hexdigest()
