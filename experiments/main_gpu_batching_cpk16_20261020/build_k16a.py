"""Bounded private build and correctness tests. NEVER launches ROS or timing."""
from pathlib import Path
import argparse
import hashlib
import json
import os
import shlex
import subprocess
import time
import xml.etree.ElementTree as ET

W = Path("/workspace/cbs_gtsam4.3")
R = W/"runs/main_gpu_batching_cpk16_20261020"
S = R/"source"
PROVIDER = W/"runs/native_covariance_cache_cpi3_20261019/release_v2"
BATCH = W/"runs/e1b4a_local_update_batching_20261012/release_2"
OLD = W/"runs/e1b_local_profile_20261012"
SNAP = W/"runs/e1b4a_snapshot_checker_20261012"
SHARED = W/"runs/shared_linearization_s4_provider_20261012/release_2"

def sha(p):
    h=hashlib.sha256()
    with Path(p).open("rb") as f:
        for b in iter(lambda:f.read(1024*1024),b""):h.update(b)
    return h.hexdigest()

def main():
    p=argparse.ArgumentParser()
    p.add_argument("variant",choices=["release","ubsan","asan"])
    p.add_argument("attempt")
    a=p.parse_args()
    out=R/(a.variant+"_"+a.attempt);out.mkdir(exist_ok=False)
    records=[]
    includes=["-I"+str(R/"snapshot_include"),"-I"+str(S),
              "-I"+str(PROVIDER.parent/"source"),
              "-I"+str(W/"runs/e1b4a_local_update_batching_20261012/authoring"),
              "-I"+str(OLD),"-I"+str(OLD/"overlay/include"),
              "-I"+str(W/"src/glim_ros1/include"),"-I"+str(W/"src/cbs/include")]
    extra=[]
    if a.variant!="release":
        extra=["-O1","-g1","-fno-omit-frame-pointer",
               "-fsanitize="+("undefined" if a.variant=="ubsan" else "address,undefined"),
               "-fno-sanitize-recover=all"]
    if a.variant=="asan": extra+=["-DEIGEN_MALLOC_ALREADY_ALIGNED=1"]
    env=os.environ.copy()
    env["LD_LIBRARY_PATH"]=":".join(map(str,[out,PROVIDER,SHARED,BATCH,OLD/"release_2",
        W/"devel/lib",Path("/opt/ros/noetic/lib"),Path("/usr/local/cuda/lib64")]))
    env["OMP_NUM_THREADS"]="1"
    env["ASAN_OPTIONS"]="detect_leaks=0:halt_on_error=1"
    env["UBSAN_OPTIONS"]="halt_on_error=1:print_stacktrace=1"
    for k in ["GLIM_SHARED_NATIVE_PROVIDER","GLIM_SHARED_NATIVE_SHADOW",
              "GLIM_REVISION_COVARIANCE_POLICY","CPK16_MAIN_GPU_BATCHING"]:
        env.pop(k,None)
    def run(name,cmd,cwd=S):
        log=out/(name+".log");start=time.monotonic()
        with log.open("x") as f:
            r=subprocess.run(cmd,cwd=cwd,env=env,stdout=f,stderr=subprocess.STDOUT)
        records.append(dict(name=name,command=cmd,cwd=str(cwd),
                            returncode=r.returncode,seconds=time.monotonic()-start))
        (out/"commands.json").write_text(json.dumps(records,indent=2)+"\n")
        print(name,r.returncode,flush=True)
        if r.returncode:
            print(log.read_text()[-10000:],flush=True)
            raise SystemExit("FAILED "+str(log))
    recipes=json.loads((PROVIDER/"commands.json").read_text())
    database=json.loads((W/"build/gtsam_points/compile_commands.json").read_text())
    entry=next(x for x in database if x["file"].endswith("/optimizers/isam2_ext.cpp"))
    base=shlex.split(entry["command"])
    compile_inputs={entry["file"]:sha(entry["file"])}
    for name in ["isam2_ext","MainGpuBatch","incremental_fixed_lag_smoother_ext_with_fallback"]:
        cmd=base.copy();cmd[cmd.index("-o")+1]=str(out/(name+".o"))
        cmd[cmd.index("-c")+1]=str(S/(name+".cpp"))
        cmd[1:1]=includes
        cmd+=extra
        run(name,cmd,entry["directory"])
    link=recipes[4]["command"].copy()
    originals={}
    for i,arg in enumerate(link):
        if arg.endswith(".o") or ".so" in arg and not arg.startswith("-"):
            ip=Path(arg) if arg.startswith("/") else Path(recipes[4]["cwd"])/arg
            # Original output is a protected baseline library too.
            if ip.exists():originals[str(ip)]=sha(ip)
        if arg=="CMakeFiles/gtsam_points.dir/src/gtsam_points/optimizers/isam2_ext.cpp.o":
            link[i]=str(out/"isam2_ext.o")
        elif arg==str(PROVIDER/"incremental_fixed_lag_smoother_ext_with_fallback.o"):
            link[i]=str(out/"incremental_fixed_lag_smoother_ext_with_fallback.o")
        elif arg==str(PROVIDER/"libgtsam_points.so.1.2.1"):
            link[i]=str(out/"libgtsam_points.so.1.2.1")
    link += [str(out/"MainGpuBatch.o")]+extra
    (out/"original_link_inputs.json").write_text(json.dumps(originals,indent=2)+"\n")
    run("provider_link",link,recipes[4]["cwd"])
    for name in ["libgtsam_points.so","libgtsam_points.so.1"]:
        (out/name).symlink_to("libgtsam_points.so.1.2.1")
    test=json.loads((W/"runs/glim_sync_compatible_batching_e1_20261011/private/test_attempt3_commands.json").read_text())[0]
    test=[str(S/"test_main_batch.cpp") if v.endswith("/cbs_local_graph_linearizer_test.cpp") else v for v in test]
    test[test.index("-o")+1]=str(out/"test_main_batch")
    test[1:1]=includes+["-L"+str(out),"-L"+str(BATCH),"-L"+str(OLD/"release_2")]
    test += [str(S/"ExactGpuSnapshot.cpp"),"-lcudart","-L/usr/local/cuda/lib64",
             "-ltbb","-le1b_profile"]+extra
    if a.variant=="asan":test+=["-fno-pie","-no-pie"]
    run("test_build",test)
    run("loader",["ldd",str(out/"test_main_batch")])
    loaded=(out/"loader.log").read_text()
    assert str(out/"libgtsam_points.so.1") in loaded,loaded
    cuda_path = next(line.split("=>",1)[1].strip().split()[0]
                     for line in loaded.splitlines() if "libgtsam_points_cuda.so.1 =>" in line)
    assert Path(cuda_path).resolve() == (OLD/"release_2/libgtsam_points_cuda.so.1.2.1").resolve()
    tests = [("main","MainBatch*"),("original","LocalLinearizer*")]
    if a.variant=="asan":tests=[("cpu","MainBatchCpu.*:LocalLinearizerCpu.*")]
    summaries={}
    for name,filt in tests:
        run(name,[str(out/"test_main_batch"),"--gtest_filter="+filt,
                  "--gtest_output=xml:"+str(out/(name+".xml"))])
        summaries[name]=ET.parse(out/(name+".xml")).getroot().attrib
    # One independent repeated process, not a source replay or benchmark.
    if a.variant=="release":
        run("repeat",[str(out/"test_main_batch"),"--gtest_filter=MainBatch*",
                      "--gtest_output=xml:"+str(out/"repeat.xml")])
        summaries["repeat"]=ET.parse(out/"repeat.xml").getroot().attrib
    assert all(sha(p)==h for p,h in originals.items()),"baseline link input changed"
    result=dict(passed=True,variant=a.variant,tests=summaries,
                library_sha256=sha(out/"libgtsam_points.so.1.2.1"),
                test_sha256=sha(out/"test_main_batch"),
                scope="K16a factor/dispatch only; no MAIN solver lifecycle, timing or dataset",
                sanitizer_scope="changed host objects/tests only; reused dependencies/CUDA kernels not instrumented")
    (out/"passed.json").write_text(json.dumps(result,indent=2)+"\n")
    print(json.dumps(result),flush=True)

if __name__=="__main__":main()
