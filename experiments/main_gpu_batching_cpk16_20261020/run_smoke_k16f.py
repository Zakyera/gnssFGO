"""One tiny observer plumbing check before real source playback."""
from pathlib import Path
import json
import os
import subprocess
W=Path("/workspace/cbs_gtsam4.3")
R=W/"runs/main_gpu_batching_cpk16_20261020"
P=Path(__file__).resolve().parent
out=P/"smoke"
out.mkdir(exist_ok=False)
probe=out/"probe"
probe.mkdir()
env=os.environ.copy()
dirs=[R/"release_1",W/"runs/native_covariance_cache_cpi3_20261019/release_v2",
      W/"runs/local_revision_covariance_cpi1_20261019",
      W/"runs/shared_linearization_s4_provider_20261012/release_2"]
env.update(LD_LIBRARY_PATH=":".join(map(str,dirs))+":"+env.get("LD_LIBRARY_PATH",""),
           LD_PRELOAD=str(P/"build_1/probe.so"),CPK16F_PROBE_ROOT=str(probe))
env.pop("CPK16_MAIN_GPU_BATCHING",None)
command=[str(P/"build_1/smoke"),str(R/"k16b_checks_4/frozen_geometry.bin")]
with (out/"stdout.log").open("x") as f:
    rc=subprocess.run(command,env=env,stdout=f,stderr=subprocess.STDOUT,timeout=30).returncode
result=dict(returncode=rc,command=command,passed=False)
if rc==0:
    rows=[json.loads(s) for s in (probe/"updates.jsonl").read_text().splitlines()]
    result["last"]=rows[-1]
    result["passed"]=len(rows)==3 and rows[-1]["prepared_hits"]>0 and rows[-1]["pair_queries"]>0
with (out/"result.json").open("x") as f:json.dump(result,f,indent=2)
print(json.dumps(result,indent=2))
if not result["passed"]:
    print((out/"stdout.log").read_text()[-6000:])
    if (probe/"failure.txt").exists():print((probe/"failure.txt").read_text())
    raise SystemExit(1)
