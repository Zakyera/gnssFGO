"""One bounded invocation per mode, stops immediately on any failure."""
from pathlib import Path
import importlib.util
import json
import subprocess
import sys
import os

W=Path("/workspace/cbs_gtsam4.3")
R=W/"runs/main_gpu_batching_cpk16_20261020"
sys.path.insert(0,str(R/"k16b_source"))
from build_k16b import environment,P,sha
from compare_k16b import compare

def main():
    build=R/("k16b_build_"+sys.argv[1])
    out=R/("k16b_checks_"+sys.argv[2]);out.mkdir(exist_ok=False)
    assert json.loads((build/"built.json").read_text())["passed"]
    results={}
    for name,exe,lib,flag in [
        ("reference","reference",P,"0"),
        ("disabled","candidate",R/"release_1","0"),
        ("enabled","candidate",R/"release_1","1"),
        ("ubsan","ubsan",R/"ubsan_1","1")]:
        data=out/(name+".txt");log=out/(name+".log")
        assert not data.exists() and not log.exists()
        geometry=out/"frozen_geometry.bin"
        assert geometry.exists()==(name!="reference")
        before_hash=sha(geometry) if geometry.exists() else None
        command=[str(build/exe),str(data),flag,str(geometry)]
        with log.open("x") as f:
            rc=subprocess.run(command,env=environment(lib),stdout=f,stderr=subprocess.STDOUT).returncode
        results[name]={"returncode":rc,"command":command,"executable_sha256":sha(build/exe),
                       "library_sha256":sha(lib/"libgtsam_points.so.1.2.1")}
        (out/"progress.json").write_text(json.dumps(results,indent=2)+"\n")
        print(name,rc,flush=True)
        print(log.read_text()[-3000:],flush=True)
        if rc:raise SystemExit("STOP: "+str(log))
        assert before_hash is None or before_hash==sha(geometry),"frozen geometry modified"
        results[name]["geometry_sha256"]=sha(geometry)
        if name!="reference":
            try:result=compare(out/"reference.txt",data,name=="disabled")
            except Exception as e:
                (out/"failed.txt").write_text(str(e)+"\n")
                raise
            results[name]["comparison"]=result
            (out/(name+"_comparison.json")).write_text(json.dumps(result,indent=2)+"\n")
            print(json.dumps(result),flush=True)
    (out/"passed.json").write_text(json.dumps(dict(passed=True,modes=results),indent=2)+"\n")
if __name__=="__main__":main()
