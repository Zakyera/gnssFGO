"""Focused test of the diagnostic assertion; no source playback."""
from pathlib import Path
import json
import os
import subprocess
P=Path(__file__).resolve().parent
R=P.parent
out=P/"cpu_copy_check"
out.mkdir(exist_ok=False)
args=json.loads((R/"k16b_build_5/commands.json").read_text())[0]["command"]
args=[str(P/"check_cpu_copy_k16f.cpp") if x.endswith("/test_main_lifecycle.cpp") else x
      for x in args if x not in ("-lgtest","-lgtest_main") and not x.endswith("/ExactGpuSnapshot.cpp")]
args[args.index("-o")+1]=str(out/"check")
with (out/"compile.log").open("x") as f:
    rc=subprocess.run(args,stdout=f,stderr=subprocess.STDOUT).returncode
(out/"command.json").write_text(json.dumps(dict(command=args,returncode=rc),indent=2))
if rc:
    print((out/"compile.log").read_text()[-6000:]);raise SystemExit(rc)
env=os.environ.copy()
env["LD_LIBRARY_PATH"]=":".join(map(str,[
    R/"release_1",R.parent/"native_covariance_cache_cpi3_20261019/release_v2",
    R.parent/"shared_linearization_s4_provider_20261012/release_2"]))+":"+env.get("LD_LIBRARY_PATH","")
result=subprocess.run([str(out/"check")],env=env,capture_output=True,text=True,timeout=20)
with (out/"stdout.log").open("x") as f:f.write(result.stdout+result.stderr)
print(result.stdout,result.stderr)
if result.returncode:raise SystemExit(result.returncode)
with (out/"result.json").open("x") as f:json.dump(json.loads(result.stdout),f,indent=2)
