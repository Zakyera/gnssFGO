import datetime, hashlib, json, stat
from pathlib import Path
H=Path(__file__).resolve().parent
E=H.parent
P=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/eth1_prior_fix_live_cpv103_20261103/prepared')
R=P/'full_on_receiver'
B=Path('/home/yeranis/results/gnssfgo_receiver_finalization_cpk19_20261021/live')
PY='/home/yeranis/.local/share/pipx/venvs/evo/bin/python'
def now():return datetime.datetime.now().astimezone().isoformat()
def read(p):return json.loads(Path(p).read_text())
def save(name,v):
    with (H/name).open('x') as f:json.dump(v,f,indent=2,allow_nan=False)
def sha(p):
    assert stat.S_ISREG(Path(p).stat().st_mode),p
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def check(entries):
    for p,v in entries.items():assert sha(p)==v,p
    return len(entries)
def tree(root):
    result={}
    for p in sorted(root.rglob('*')):
        mode=p.lstat().st_mode
        assert stat.S_ISREG(mode) or stat.S_ISDIR(mode),p
        if stat.S_ISREG(mode):result[str(p.relative_to(root))]=sha(p)
    return result
def tree_digest(entries):
    return hashlib.sha256(json.dumps(entries,sort_keys=True,separators=(',',':')).encode()).hexdigest()
