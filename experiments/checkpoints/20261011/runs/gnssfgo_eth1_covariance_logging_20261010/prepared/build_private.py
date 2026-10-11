#!/usr/bin/env python3
"""Compile diagnostic GLIM wrapper only, into a NEW private library directory.

Run inside cbsms_ws_gpu; never changes the catkin runtime or old artifacts.
"""
import hashlib
import json
import shlex
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parent
WS = ROOT.parents[2]
BUILD = WS / 'build/glim_ros'
TARGET = BUILD / 'CMakeFiles/glim_ros.dir'
OUT = ROOT / 'private_build'
OUT.mkdir(exist_ok=False)
flags = {}
for line in (TARGET / 'flags.make').read_text().splitlines():
    if line.startswith('CXX_') and ' = ' in line:
        k, v = line.split(' = ', 1)
        flags[k] = shlex.split(v)
commands = []
for name in ['glim_ros', 'cbs_bridge']:
    source = WS / f'src/glim_ros1/src/glim_ros/{name}.cpp'
    command = ['/usr/bin/c++', *flags['CXX_DEFINES'], *flags['CXX_INCLUDES'],
               *flags['CXX_FLAGS'], '-c', str(source), '-o', str(OUT / (name + '.o'))]
    commands.append(command)
    with (OUT / (name + '.log')).open('x') as log:
        subprocess.run(command, cwd=BUILD, stdout=log, stderr=subprocess.STDOUT, check=True)
command = shlex.split((TARGET / 'link.txt').read_text())
command[command.index('-o') + 1] = str(OUT / 'libglim_ros.so')
for i, value in enumerate(command):
    for name in ['glim_ros', 'cbs_bridge']:
        if value == f'CMakeFiles/glim_ros.dir/src/glim_ros/{name}.cpp.o':
            command[i] = str(OUT / (name + '.o'))
commands.append(command)
with (OUT / 'link.log').open('x') as log:
    subprocess.run(command, cwd=BUILD, stdout=log, stderr=subprocess.STDOUT, check=True)
with (OUT / 'build.json').open('x') as out:
    json.dump({'commands': commands, 'sha256': hashlib.sha256(
        (OUT / 'libglim_ros.so').read_bytes()).hexdigest()}, out, indent=2)
print('PRIVATE_BUILD_COMPLETE', OUT / 'libglim_ros.so', flush=True)
