"""Reuse the retained no-data startup/owned-shutdown controller, no bag."""
from pathlib import Path
import hashlib
import json

HERE=Path(__file__).resolve().parent
ROOT=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/e1b4a_local_update_batching_20261012')
source=HERE.parent/'e1b_startup_fix_20261012/debug_startup.py'
text=source.read_text()
changes={
    "RUN=WS/'runs/e1b_startup_fix_20261012'":"RUN=WS/'runs/e1b4a_local_update_batching_20261012'",
    "LIB=WS/'runs/e1b_local_profile_20261012/release_2'":"LIB=RUN/'release_2'",
    'PORT=11407':'PORT=11411',
    "if a.profile: extra+=":"extra+='export CBS_E1B_LOCAL_UPDATE_GPU_BATCHING=1 && '\n        if a.profile: extra+=",
    "assert inside(a.library/'libglim_ros.so') in maps":
        "assert inside(a.library/'libglim_ros.so') in maps\n            assert inside(a.library/'libgtsam.so.4.3a1') in maps",
}
for before,after in changes.items():
    assert text.count(before)==1,before
    text=text.replace(before,after)
with (ROOT/'startup.py').open('x') as f:f.write(text)
(ROOT/'startup_provenance.json').write_text(json.dumps(dict(source=str(source),sha256=hashlib.sha256(source.read_bytes()).hexdigest(),replacements=changes),indent=2)+'\n')
