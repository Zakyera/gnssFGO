"""Run only the isolated numerical adapter; no ROS or estimator launch."""
import json
import subprocess
import time
from pathlib import Path

root=Path(__file__).resolve().parent
records=[]
for name in ['aligned','aligned_repeat']:
    output=root/(name+'.txt')
    logfile=root/('validated_'+name+'.log')
    assert not output.exists() and not logfile.exists()
    command=[str(root/'mean_adapter'),str(root/'imu.txt'),str(root/'queries.txt'),str(output)]
    start=time.monotonic()
    with logfile.open('x') as stream:
        result=subprocess.run(command,stdout=stream,stderr=subprocess.STDOUT)
    records.append(dict(command=command,returncode=result.returncode,seconds=time.monotonic()-start))
    if result.returncode:
        with (root/'failed_execution.json').open('x') as stream: json.dump(records,stream,indent=2)
        raise SystemExit(result.returncode)
with (root/'execution.json').open('x') as stream: json.dump(records,stream,indent=2)
print(json.dumps(records,indent=2))
