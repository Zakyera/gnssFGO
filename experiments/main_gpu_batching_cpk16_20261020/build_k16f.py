"""Compile removable observer plus bounded three-update plumbing test only."""
from pathlib import Path
import json
import os
import subprocess
P=Path(__file__).resolve().parent
R=P.parent
out=P/"build_1"
out.mkdir(exist_ok=False)
base=json.loads((R/"k16b_build_5/commands.json").read_text())[0]["command"]
base=[str(P/"probe_k16f.cpp") if x.endswith("/test_main_lifecycle.cpp")
      else str(P/"ExactGpuSnapshot.cpp") if x.endswith("/ExactGpuSnapshot.cpp")
      else x for x in base if x not in ("-lgtest","-lgtest_main")]
base[1:1]=["-I"+str(P)]
base[base.index("-o")+1]=str(out/"probe.so")
base+=["-shared","-ldl"]
commands=[]
for name,args in [("probe",base),("smoke",[
    str(P/"smoke_k16f.cpp") if x==str(P/"probe_k16f.cpp")
    else str(out/"smoke") if x==str(out/"probe.so") else x
    for x in base if x!="-shared"])]:
    with (out/(name+".log")).open("x") as f:
        rc=subprocess.run(args,stdout=f,stderr=subprocess.STDOUT).returncode
    commands.append(dict(name=name,command=args,returncode=rc))
    (out/"commands.json").write_text(json.dumps(commands,indent=2))
    if rc:
        print((out/(name+".log")).read_text()[-12000:]);raise SystemExit(rc)
print("Observer and plumbing executable built; frozen estimator unchanged")
