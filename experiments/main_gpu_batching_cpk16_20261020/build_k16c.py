"""Compile diagnostic only against immutable K16b-tested DSOs."""
from pathlib import Path
import json
import shutil
import subprocess
import sys

W=Path("/workspace/cbs_gtsam4.3")
R=W/"runs/main_gpu_batching_cpk16_20261020"
sys.path.insert(0,str(R/"k16b_source"))
from build_k16b import environment,P,sha

out=R/"k16c_build_1"
out.mkdir(exist_ok=False)
assert sha(R/"release_1/libgtsam_points.so.1.2.1")=="6f71292d29d974a0555abd9fda6b2370fdab81ca634a4e1343c216d2a762d2ca"
assert sha(P/"libgtsam_points.so.1.2.1")=="3f23e6115e34acc5a2f2b965b15bd4c651897c889a6c38a0a52cdb3855c0ceb7"
shutil.copyfile(R/"k16c_source/test_timing.cpp",out/"test_timing.cpp")
shutil.copyfile(R/"k16b_build_5/test_main_lifecycle.cpp",out/"test_main_lifecycle.cpp")
records=[]
for old in json.loads((R/"k16b_build_5/commands.json").read_text())[:2]:
    mode=old["mode"];lib=P if mode=="reference" else R/"release_1"
    argv=[str(out/"test_timing.cpp") if x.endswith("/test_main_lifecycle.cpp") else x for x in old["command"]]
    argv[argv.index("-o")+1]=str(out/mode)
    argv+=["-lcrypto"]
    with (out/(mode+"_build.log")).open("x") as log:
        rc=subprocess.run(argv,env=environment(lib),stdout=log,stderr=subprocess.STDOUT).returncode
    records.append(dict(mode=mode,command=argv,returncode=rc))
    (out/"commands.json").write_text(json.dumps(records,indent=2)+"\n")
    print(mode,rc,flush=True)
    if rc:
        print((out/(mode+"_build.log")).read_text()[-10000:],flush=True)
        raise SystemExit(rc)
    loaded=subprocess.check_output(["ldd",str(out/mode)],env=environment(lib),text=True)
    (out/(mode+"_ldd.txt")).write_text(loaded)
    actual=next(x.split("=>")[1].strip().split()[0] for x in loaded.splitlines()
                if "libgtsam_points.so.1 =>" in x)
    assert Path(actual).resolve()==(lib/"libgtsam_points.so.1.2.1").resolve()
(out/"built.json").write_text(json.dumps(dict(passed=True,executables={
    m:sha(out/m) for m in ["reference","candidate"]}),indent=2)+"\n")
