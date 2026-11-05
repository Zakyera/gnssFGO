"""Build ONLY the input adapter, borrowing existing GLIM ROS compile/link flags."""
from pathlib import Path
import hashlib,json,shlex,subprocess
P=Path(__file__).resolve().parent; W=Path('/workspace/cbs_gtsam4.3')
B=P/'driver_build_4'; assert not B.exists();B.mkdir()
flags={line.split(' = ',1)[0]:shlex.split(line.split(' = ',1)[1])
       for line in (W/'build/glim_ros/CMakeFiles/glim_rosbag.dir/flags.make').read_text().splitlines() if ' = ' in line}
compile=['/usr/bin/c++',*flags['CXX_DEFINES'],*flags['CXX_INCLUDES'],*flags['CXX_FLAGS'],
         '-c',str(P/'paced_k16h.cpp'),'-o',str(B/'driver.o')]
link=shlex.split((W/'build/glim_ros/CMakeFiles/glim_rosbag.dir/link.txt').read_text())
link=[str(B/'driver.o') if x=='CMakeFiles/glim_rosbag.dir/src/glim_rosbag.cpp.o' else x for x in link]
link[link.index('-o')+1]=str(B/'paced_input')
with (B/'commands.json').open('x') as f:json.dump([compile,link],f,indent=2)
with (B/'build.log').open('x') as f:
    subprocess.run(compile,check=True,stdout=f,stderr=subprocess.STDOUT)
    subprocess.run(link,check=True,stdout=f,stderr=subprocess.STDOUT)
with (B/'complete.json').open('x') as f:json.dump(dict(passed=True,sha256=hashlib.sha256((B/'paced_input').read_bytes()).hexdigest()),f)
print('Private input adapter built; estimator and observer binaries unchanged.')
