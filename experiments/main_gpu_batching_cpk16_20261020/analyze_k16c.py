"""Analyze all predeclared timing samples; no estimator work or selection."""
from pathlib import Path
import hashlib
import json
import math
import statistics

H=Path(__file__).resolve().parent
R=Path("/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/main_gpu_batching_cpk16_20261020")
root=R/"k16c_checks_1"
receipt=json.loads((root/"passed.json").read_text());assert receipt["passed"]
def stats(v):
    values=sorted(v)
    def q(p):
        x=p*(len(values)-1);i=math.floor(x);j=math.ceil(x)
        return values[i]+(values[j]-values[i])*(x-i)
    return dict(n=len(v),median=statistics.median(v),p95=q(.95),max=max(v),min=min(v),
                mean=statistics.mean(v))
rows={};memory={};maxima={}
for name,run in receipt["runs"].items():
    assert run["returncode"]==0
    for key,value in run.get("comparison",{}).get("max_absolute_differences",{}).items():
        maxima[key]=max(value,maxima.get(key,0))
    if name.startswith("audit"):continue
    _,rep,mode=name.split("_");rep=int(rep)
    for line in (root/(name+".timing")).read_text().splitlines():
        w,cycle,phase,ms=line.split();key=(int(w),rep,mode,int(cycle),int(phase))
        assert key not in rows and math.isfinite(float(ms)) and float(ms)>0
        rows[key]=float(ms)
    for line in (root/(name+".log")).read_text().splitlines():
        if line.startswith("MEMORY "):
            _,w,device,rss=line.split()
            memory[int(w),rep,mode]=dict(device_observed_delta_bytes=int(device),
                                       process_maxrss_kib=int(rss))
expected={(w,r,m,c,p) for w in [5,50] for r in range(6) for m in ["reference","enabled"]
          for c in range(20) for p in range(4)}
assert rows.keys()==expected,("missing/extra timing",len(rows),len(expected))
for w,r,m,c,p in rows:
    if p==3:assert abs(rows[w,r,m,c,3]-sum(rows[w,r,m,c,i] for i in range(3)))<1e-8
workloads={}
for w in [5,50]:
    modes={}
    for m in ["reference","enabled"]:
        modes[m]=dict(cycle_ms=stats([rows[w,r,m,c,3] for r in range(6) for c in range(20)]),
          phase_ms={str(p):stats([rows[w,r,m,c,p] for r in range(6) for c in range(20)])
                    for p in range(3)},
          rounds=[stats([rows[w,r,m,c,3] for c in range(20)]) for r in range(6)],
          memory=[memory[w,r,m] for r in range(6)])
    a,b=modes["reference"],modes["enabled"]
    modes["ratio"]={k:b["cycle_ms"][k]/a["cycle_ms"][k] for k in ["median","p95","mean","max"]}
    modes["paired_ratios"]=[dict(round=r,first="reference" if r%2==0 else "enabled",
      median=b["rounds"][r]["median"]/a["rounds"][r]["median"],
      p95=b["rounds"][r]["p95"]/a["rounds"][r]["p95"]) for r in range(6)]
    workloads[str(w)]=modes
audit=receipt["runs"]["audit_enabled"]["comparison"]
audit_load={w:{"requests":[],"prepared":[],"active_gpu":[],"factor_slots":[]} for w in [5,50]}
for line in (root/"audit_enabled.txt").read_text().splitlines():
    fields=line.split();parts=fields[0].split("/")
    if len(parts)!=4 or parts[1]=="final":continue
    w,k,phase=map(int,parts[:3])
    if k<w+5:continue
    values=list(map(int,fields[2:]))
    if parts[3]=="profile":
        audit_load[w]["requests"].append(values[0])
        audit_load[w]["prepared"].append(values[7])
    elif parts[3]=="load":
        assert values[0]==3*w,("unbounded/window state count",w,k,values)
        audit_load[w]["factor_slots"].append(values[1])
        audit_load[w]["active_gpu"].append(values[2])
for w,values in audit_load.items():
    assert all(len(v)==60 for v in values.values())
    workloads[str(w)]["untimed_per_update_load"]={k:stats(v) for k,v in values.items()}
result=dict(numerical_passed=True,timing_rows=len(rows),update_cycles=480,
  maximum_differences=maxima,workloads=workloads,
  untimed_reference_counters=audit["reference_counters"],
  untimed_candidate_counters=audit["candidate_counters"],
  performance_target_passed=all(workloads["50"]["ratio"][k]<=.8 for k in ["median","p95"]),
  cpu=json.loads((root/"plan.json").read_text())["cpu"],
  geometry_sha256=json.loads((root/"plan.json").read_text())["geometry_sha256"],
  caveats=["Synthetic 864-point input, not ETH1 or complete GLIM throughput.",
           "Cycle sums three synchronized MAIN updates; input prep and export excluded.",
           "Native shared-provider work included; profiling disabled in timing.",
           "Original synchronous warning output is retained, so times include that existing path cost.",
           "Memory is process high-water/sample device delta, not isolated peak allocation.",
           "Wrapper does not expose exact per-batch sizes; prepared calls are per-update totals.",
           "Separate Kimera revision/receiver-retention mismatch unresolved."])
with (H/"RESULT_K16C.json").open("x") as f:json.dump(result,f,indent=2)
print(json.dumps({k:v for k,v in result.items() if k!="workloads"},indent=2))
for w,values in workloads.items():
    print(w,json.dumps({m:values[m]["cycle_ms"] for m in ["reference","enabled"]}),
          json.dumps(values["ratio"]))
