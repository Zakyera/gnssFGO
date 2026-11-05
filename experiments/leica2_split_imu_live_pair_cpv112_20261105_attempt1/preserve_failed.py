"""Metadata-only special-file check. Never opens FIFOs or follows symlinks."""
import json,os,stat
from pathlib import Path
def verify_special():
    h=Path(__file__).resolve().parent
    for name,v in json.loads((h/'preserved_failed_special.json').read_text()).items():
        p=Path(name);s=p.lstat();assert stat.S_IMODE(s.st_mode)==v['mode'],name
        if v['kind']=='symlink':assert stat.S_ISLNK(s.st_mode) and os.readlink(p)==v['target'],name
        else:assert v['kind']=='fifo' and stat.S_ISFIFO(s.st_mode),name
