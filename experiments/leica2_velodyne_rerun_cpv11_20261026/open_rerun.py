"""Open this saved visualization only, with localhost-only viewer connectivity."""
import json
import subprocess
import time
from datetime import datetime
from pathlib import Path
H=Path(__file__).resolve().parent
assert json.loads((H/'recording_checks.json').read_text())['passed']
receipt=H/'viewer_launch.json'
assert not receipt.exists(), 'Viewer was already launched; use its existing window or open the RRD manually.'
cmd=['/home/yeranis/.local/bin/rerun',str(H/'leica2_velodyne_first_published.rrd'),
     '--bind','127.0.0.1','--port','auto','--memory-limit','2GB','--window-size','1600x1100']
with (H/'viewer.log').open('x') as log:
    p=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
time.sleep(3)
data={'pid':p.pid,'started':datetime.now().astimezone().isoformat(),'command':cmd,'exit_after_3s':p.poll()}
with receipt.open('x') as f:json.dump(data,f,indent=2)
print(json.dumps(data,indent=2))
assert data['exit_after_3s'] is None,'Viewer exited; inspect log.'
