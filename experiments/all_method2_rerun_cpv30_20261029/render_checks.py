"""Native screenshot-only visual checks; no long-lived viewer or experiment."""
import json
import os
import subprocess
import sys
from pathlib import Path

H = Path(__file__).resolve().parent
OUT = Path('/home/yeranis/results/rerun_method2_all_sequences_20261029')
cases = json.loads((H/'input_checks.json').read_text())['cases']
results = []
suffix = '_host' if '--host' in sys.argv else ''
for case in cases:
    image = H/(case+'_preview.png')
    assert not image.exists()
    cmd = ['/home/yeranis/.local/bin/rerun',str(OUT/(case+'_method2.rrd')),
        '--bind','127.0.0.1','--port','auto','--memory-limit','3GB',
        '--window-size','1600x1100','--screenshot-to',str(image)]
    with (H/(case+'_screenshot'+suffix+'.log')).open('x') as log:
        result = subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT,
            env=dict(os.environ),timeout=55)
    receipt = dict(case=case,command=cmd,returncode=result.returncode,
        screenshot=str(image),image_exists=image.exists(),
        image_bytes=image.stat().st_size if image.exists() else 0)
    results.append(receipt)
    with (H/(case+'_screenshot'+suffix+'.json')).open('x') as f: json.dump(receipt,f,indent=2)
    print(json.dumps(receipt),flush=True)
    assert receipt['image_bytes'] > 10000, 'Screenshot absent; inspect log'
with (H/('render_receipts'+suffix+'.json')).open('x') as f: json.dump(results,f,indent=2)
