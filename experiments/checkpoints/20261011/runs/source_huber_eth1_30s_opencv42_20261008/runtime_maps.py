"""Read-only process/library provenance in the ROS container."""
from pathlib import Path
import json
import sys
root=Path('/workspace/source_huber_20261008')
run=sys.argv[1]
found={}
for p in Path('/proc').glob('[0-9]*'):
    try:
        env=(p/'environ').read_bytes()
        if ('CBSMS_RUN_NAME='+run+'\0').encode() not in env: continue
        cmd=(p/'cmdline').read_bytes().replace(b'\0',b' ').decode()
        exe=str((p/'exe').resolve())
        if exe.endswith('/glim_rosnode'): kind='glim'
        elif exe.endswith('/kimera_vio_ros_node'): kind='kimera'
        else: continue
        maps=(p/'maps').read_text()
        libs=sorted({l.split()[-1] for l in maps.splitlines() if '.so' in l})
        required=['/workspace/source_huber_20261008/install/lib/libcbs.so']
        required += ['/workspace/source_huber_20261008/devel/lib/libglim_ros.so'] if kind=='glim' else ['/workspace/source_huber_20261008/build/kimera_vio_opencv42/libkimera_vio.so']
        assert all(f in libs for f in required), (kind,required,libs)
        assert not any(l.endswith(('/libcbs.so','/libkimera_vio.so','/libglim_ros.so')) and l.startswith('/workspace/cbs_gtsam4.3/devel') for l in libs),libs
        if kind=='kimera':
            assert any('libopencv_core.so.4.2' in l for l in libs), libs
            assert not any('libopencv_' in l and '.so.3.4' in l for l in libs), libs
        found[kind]=dict(pid=int(p.name),command=cmd,executable=exe,libraries=libs,
            loss_env=[s.decode() for s in env.split(b'\0') if s.startswith((b'CBS_GLIM_SOURCE_LOSS=',b'CBS_KIMERA_SOURCE_LOSS=',b'CBS_SOURCE_LOSS_AUDIT='))])
    except (FileNotFoundError,PermissionError,ProcessLookupError): pass
print(json.dumps(found,indent=2))
sys.exit(0 if set(found)=={'glim','kimera'} else 2)
