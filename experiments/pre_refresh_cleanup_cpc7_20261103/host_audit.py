"""Host-visible read-only safety check; never stop any process/container."""
from pathlib import Path
import datetime,json,os,subprocess,sys,time
H=Path(__file__).resolve().parent
label=sys.argv[1]
assert label in ('before','admission','after')
scope=json.loads((H/'scope.json').read_text())['roots']
roots=tuple(p+'/' for p in scope)
jobs=[];opened=[];permissions=0;processes=0
for p in Path('/proc').iterdir():
    if not p.name.isdigit():continue
    try:
        args=p.joinpath('cmdline').read_bytes().decode(errors='replace').rstrip('\0').split('\0')
        if not args or not args[0]:continue
        processes+=1
        # Match executable/script arguments, not shell descriptions or this check.
        name=Path(args[0]).name
        interesting=(name in {'roslaunch','roscore','rosmaster','rosbag','glim_rosnode','kimera_vio_ros','live','live_trace'} or any(Path(x).name=='controller.py' for x in args[1:3]))
        if interesting:jobs.append({'pid':int(p.name),'args':args})
        for fd in (p/'fd').iterdir():
            try:target=os.readlink(fd)
            except (FileNotFoundError,PermissionError):continue
            if target.startswith(roots):opened.append({'pid':int(p.name),'fd':fd.name,'path':target})
    except FileNotFoundError:continue
    except PermissionError:permissions+=1
raw=subprocess.run(['docker','inspect','--format','{{.Name}} {{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'],text=True,capture_output=True,check=True).stdout
passed=not jobs and not opened and len(raw.splitlines())==2 and all(x.endswith(' exited') for x in raw.splitlines())
record={'at':datetime.datetime.now().astimezone().isoformat(),'epoch':time.time(),'passed':passed,'processes_seen':processes,'inaccessible_processes':permissions,'experiment_jobs':jobs,'open_old_run_handles':opened,'containers':raw}
with (H/({'before':'host_safety.json','admission':'host_admission.json','after':'host_after.json'}[label])).open('x') as f:json.dump(record,f,indent=2)
print(json.dumps(record,indent=2));assert passed
