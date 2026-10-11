import json,subprocess
from pathlib import Path
HERE=Path(__file__).resolve().parent
ROOT=Path('/home/yeranis/results/gnssfgo_e1_e2_correctness_audit_20261011')
assert json.loads((ROOT/'off2_comparison.json').read_text())['valid']
record=ROOT/'cpu_cache_probe.json';assert not record.exists()
binary=ROOT/'cpu_cache_probe'
command=['c++','-std=c++20','-O3','-I/usr/include/eigen3',str(HERE/'cpu_cache_probe.cpp'),'-o',str(binary)]
subprocess.run(command,check=True)
commands=[['taskset','-c','2',str(binary)],['taskset','-c','8',str(binary)],[str(binary)]]
results=[dict(command=c,output=subprocess.check_output(c,text=True).strip()) for c in commands]
with record.open('x') as f:json.dump(dict(build=command,results=results),f,indent=2)
print(json.dumps(results,indent=2))
