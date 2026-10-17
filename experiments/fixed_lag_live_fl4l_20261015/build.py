"""Inside container: one private TU compile/link, preserving all existing objects."""
import json,subprocess
from pathlib import Path
root=Path('/workspace/cbs_gtsam4.3/runs/fixed_lag_same_graph_fl4l_20261015')
commands=json.loads((root/'build_commands.json').read_text())
for step in ['compile','link']:
    with (root/(step+'.log')).open('w') as log:
        subprocess.run(commands[step],cwd=commands['cwd'],stdout=log,stderr=subprocess.STDOUT,check=True)
    print(step+' complete',flush=True)
