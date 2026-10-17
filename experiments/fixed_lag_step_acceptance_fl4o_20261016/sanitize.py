"""Focused overlay ASan/UBSan; linked historical libraries are not rebuilt."""
import json,os,subprocess
from pathlib import Path
R=Path(__file__).resolve().parent
c=json.loads((R/'actual_test_commands.json').read_text())
env=os.environ.copy()
env['LD_LIBRARY_PATH']=str(R/'private_build')+':'+env.get('LD_LIBRARY_PATH','')
# Prior FL4k identified 73 bytes in existing non-instrumented library paths.
# This gate covers address/UB errors, NOT whole-library/leak certification.
env['ASAN_OPTIONS']='detect_leaks=0:halt_on_error=1'
env['UBSAN_OPTIONS']='halt_on_error=1:print_stacktrace=1'
def run(cmd,name):
    name=os.environ.get('FL4O_SAN_TAG','')+name
    with (R/(name+'.log')).open('x') as log:
        p=subprocess.run(cmd,env=env,stdout=log,stderr=subprocess.STDOUT)
    print(name,p.returncode,flush=True)
    if p.returncode:raise SystemExit(p.returncode)
def command(args):
    return [str(R/'test_san.o') if a==str(R/'test.o') else str(R/'test_san') if a==str(R/'test') else a for a in args if a!='-O3']
flags=['-O1','-g1','-fsanitize=address,undefined','-fno-sanitize-recover=all','-fno-omit-frame-pointer']
# Match the pinned unsanitized Eigen ABI, as in the retained FL4k sanitizer
# build. ASan's auto-detection otherwise uses handmade aligned allocations
# while the library frees returned dynamic Eigen vectors with plain free().
run(command(c['compile'])+flags+['-fno-pie','-DEIGEN_MALLOC_ALREADY_ALIGNED=1'],'san_compile')
run(command(c['link'])+flags+['-no-pie'],'san_link')
run([str(R/'test_san')],'san_run')
(R/'sanitizer_passed.json').write_text(json.dumps({'passed':True,'address_undefined':True,'leaks_checked':False,'linked_libraries_instrumented':False}))
