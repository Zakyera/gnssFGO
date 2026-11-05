"""Build only bounded K16b diagnostics against frozen libraries, not estimators."""
from pathlib import Path
import hashlib
import json
import os
import subprocess
import sys
import shutil

W=Path("/workspace/cbs_gtsam4.3")
R=W/"runs/main_gpu_batching_cpk16_20261020"
P=W/"runs/native_covariance_cache_cpi3_20261019/release_v2"
S=W/"runs/shared_linearization_s4_provider_20261012/release_2"
B=W/"runs/e1b4a_local_update_batching_20261012/release_2"
OLD=W/"runs/e1b_local_profile_20261012"

def sha(p):
    h=hashlib.sha256()
    with Path(p).open("rb") as f:
        for b in iter(lambda:f.read(1024*1024),b""):h.update(b)
    return h.hexdigest()

def environment(lib):
    env=os.environ.copy()
    env["LD_LIBRARY_PATH"]=":".join(map(str,[lib,P,S,B,OLD/"release_2",W/"devel/lib",
      Path("/opt/ros/noetic/lib"),Path("/usr/local/cuda/lib64")]))
    env["OMP_NUM_THREADS"]="1"
    env["UBSAN_OPTIONS"]="halt_on_error=1:print_stacktrace=1"
    for k in list(env):
        if k.startswith(("GLIM_SHARED_NATIVE","GLIM_REVISION_COVARIANCE","CBS_E1B_","CPK16_")):
            env.pop(k,None)
    return env

def main():
    attempt=sys.argv[1];out=R/("k16b_build_"+attempt);out.mkdir(exist_ok=False)
    unit=sys.argv[2] if len(sys.argv)>2 else "test_main_lifecycle.cpp"
    assert unit in ("test_main_lifecycle.cpp","test_incremental.cpp")
    shutil.copyfile(R/"k16b_source"/unit,out/unit)
    # A prior gate candidate is immutable. This script never links a new DSO.
    assert sha(R/"release_1/libgtsam_points.so.1.2.1")=="6f71292d29d974a0555abd9fda6b2370fdab81ca634a4e1343c216d2a762d2ca"
    assert sha(P/"libgtsam_points.so.1.2.1")=="3f23e6115e34acc5a2f2b965b15bd4c651897c889a6c38a0a52cdb3855c0ceb7"
    records=[]
    for mode in ["reference","candidate","ubsan"]:
        lib=P if mode=="reference" else R/("ubsan_1" if mode=="ubsan" else "release_1")
        argv=json.loads((W/"runs/glim_sync_compatible_batching_e1_20261011/private/test_attempt3_commands.json").read_text())[0]
        argv=[str(R/"k16b_source"/unit) if x.endswith("/cbs_local_graph_linearizer_test.cpp") else x for x in argv]
        argv[argv.index("-o")+1]=str(out/mode)
        argv[1:1]=["-I"+str(R/"snapshot_include"),"-I"+str(R/"source"),
          "-I"+str(P.parent/"source"),"-I"+str(OLD),"-I"+str(OLD/"overlay/include"),
          "-I"+str(B.parent/"authoring"),"-L"+str(lib),"-L"+str(B),"-L"+str(OLD/"release_2")]
        argv += [str(R/"source/ExactGpuSnapshot.cpp"),"-lcudart","-L/usr/local/cuda/lib64","-ltbb","-le1b_profile"]
        if mode!="reference":argv+=["-DK16_CANDIDATE"]
        if mode=="ubsan":
            argv+=["-O1","-g1","-fno-omit-frame-pointer","-fsanitize=undefined","-fno-sanitize-recover=all"]
        with (out/(mode+"_build.log")).open("x") as log:
            rc=subprocess.run(argv,env=environment(lib),stdout=log,stderr=subprocess.STDOUT).returncode
        records.append(dict(mode=mode,command=argv,returncode=rc))
        (out/"commands.json").write_text(json.dumps(records,indent=2)+"\n")
        print(mode,rc,flush=True)
        if rc:
            print((out/(mode+"_build.log")).read_text()[-12000:],flush=True)
            raise SystemExit(rc)
        loaded=subprocess.check_output(["ldd",str(out/mode)],env=environment(lib),text=True)
        (out/(mode+"_ldd.txt")).write_text(loaded)
        actual=next(x.split("=>")[1].strip().split()[0] for x in loaded.splitlines()
                    if "libgtsam_points.so.1 =>" in x)
        assert Path(actual).resolve()==(lib/"libgtsam_points.so.1.2.1").resolve()
    (out/"built.json").write_text(json.dumps({"passed":True,"executables":
      {m:sha(out/m) for m in ["reference","candidate","ubsan"]}},indent=2)+"\n")

if __name__=="__main__":main()
