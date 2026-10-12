"""Refuse overlapping compiler/estimator work in the experiment container."""
from pathlib import Path
import datetime,json,platform,subprocess
ROOT=Path('/workspace/cbs_gtsam4.3/runs/shared_linearization_s4_certificate_20261012')
names=[n.strip() for n in subprocess.check_output(['ps','-eo','comm='],text=True).splitlines()]
blocked=[n for n in names if n in {'cc1plus','cc1','clang','clang++','g++','c++','glim_rosnode','vio_ros','rosbag'} or n.startswith('gnssfgo_')]
result=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),platform=platform.platform(),
    cpu=sorted(set(l for l in Path('/proc/cpuinfo').read_text().splitlines() if l.startswith('model name'))),
    governors=sorted(set(p.read_text().strip() for p in Path('/sys/devices/system/cpu').glob('cpu[0-9]*/cpufreq/scaling_governor'))),
    overlapping_work=blocked,unlocked_desktop_clocks=True)
assert not blocked,blocked
with (ROOT/'timing_environment.json').open('x') as f:json.dump(result,f,indent=2)
print(json.dumps(result))
