"""Untimed admission then six alternating paired repetitions. No retries."""
from pathlib import Path
import json
import os
import subprocess
import sys
import time
W=Path("/workspace/cbs_gtsam4.3")
R=W/"runs/main_gpu_batching_cpk16_20261020"
sys.path.insert(0,str(R/"k16b_source"))
from build_k16b import environment,P,sha
from compare_k16b import compare

out=R/"k16c_checks_1";out.mkdir(exist_ok=False)
build=R/"k16c_build_1"
assert json.loads((build/"built.json").read_text())["passed"]
geometry=R/"k16b_checks_4/frozen_geometry.bin"
geometry_hash=sha(geometry)
cpu=min(os.sched_getaffinity(0))
results={}
plan=[("audit_reference","reference",False,True),
      ("audit_disabled","candidate",False,True),
      ("audit_enabled","candidate",True,True)]
for rep in range(6):
    order=[False,True] if rep%2==0 else [True,False]
    for enabled in order:
        plan.append((f"timing_{rep}_{'enabled' if enabled else 'reference'}",
                     "candidate" if enabled else "reference",enabled,False))
(out/"plan.json").write_text(json.dumps(dict(cpu=cpu,commands=plan,
    geometry_sha256=geometry_hash),indent=2)+"\n")
for name,exe,enabled,audit in plan:
    lib=P if exe=="reference" else R/"release_1"
    command=["taskset","-c",str(cpu),str(build/exe),str(out/(name+".txt")),
             str(out/(name+".timing")),str(int(enabled)),str(int(audit)),str(geometry)]
    before=time.monotonic()
    with (out/(name+".log")).open("x") as log:
        rc=subprocess.run(command,env=environment(lib),stdout=log,
                          stderr=subprocess.STDOUT,timeout=240).returncode
    record=dict(command=command,returncode=rc,wall_seconds=time.monotonic()-before,
                executable_sha256=sha(build/exe),library_sha256=sha(lib/"libgtsam_points.so.1.2.1"))
    results[name]=record
    (out/"progress.json").write_text(json.dumps(results,indent=2)+"\n")
    print(name,rc,round(record["wall_seconds"],3),flush=True)
    print((out/(name+".log")).read_text()[-1200:],flush=True)
    assert sha(geometry)==geometry_hash,"frozen input changed"
    if rc:raise SystemExit("STOP: "+name)
    if name!="audit_reference":
        try:
            record["comparison"]=compare(out/"audit_reference.txt",out/(name+".txt"),not enabled)
        except Exception as exc:
            (out/"failed.txt").write_text(name+"\n"+str(exc)+"\n")
            raise
        (out/(name+"_comparison.json")).write_text(json.dumps(record["comparison"],indent=2)+"\n")
        print("comparison passed",record["comparison"]["max_absolute_differences"],flush=True)
        if name=="audit_enabled":
            counts=record["comparison"]
            for workload in ["5","50"]:
                a=counts["reference_counters"][workload]
                b=counts["candidate_counters"][workload]
                assert a[0]>0 and a[0]==a[1] and a[7]==0,(workload,a)
                assert b[0]==a[0] and b[1]==0 and b[7]==a[0],(workload,a,b)
                assert a[4:7]==b[4:7],("solver work changed",workload,a,b)
    (out/"progress.json").write_text(json.dumps(results,indent=2)+"\n")
(out/"passed.json").write_text(json.dumps(dict(passed=True,runs=results),indent=2)+"\n")
