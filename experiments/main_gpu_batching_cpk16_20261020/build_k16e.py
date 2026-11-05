"""Build only removable diagnostic preload, using the prior ABI recipe."""
from pathlib import Path
import json
import os
import subprocess
P=Path(__file__).resolve().parent
R=P.parent
out=P/"build"
out.mkdir(exist_ok=False)
args=json.loads((R/"k16b_build_5/commands.json").read_text())[0]["command"]
args=[str(P/"probe_k16e.cpp") if x.endswith("/test_main_lifecycle.cpp")
      else str(P/"ExactGpuSnapshot.cpp") if x.endswith("/ExactGpuSnapshot.cpp")
      else x for x in args if x not in ("-lgtest","-lgtest_main")]
args[args.index("-o")+1]=str(out/"probe.so")
args[1:1]=["-I"+str(P)]
args+=["-shared","-ldl"]
with (out/"compile.log").open("x") as log:
    rc=subprocess.run(args,stdout=log,stderr=subprocess.STDOUT).returncode
(out/"command.json").write_text(json.dumps(dict(command=args,returncode=rc),indent=2))
if rc:
    print((out/"compile.log").read_text()[-10000:])
    raise SystemExit(rc)
print("Diagnostic-only probe built")
