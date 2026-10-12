"""Host controller: GLIM-only, no data, exact frozen params, owned cleanup."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shlex
import signal
import subprocess
import time
import xmlrpc.client
import yaml

HERE=Path(__file__).resolve().parent
WS=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
RUN=WS/'runs/e1b_startup_fix_20261012'
FAILED=WS/'runs/gnssfgo_e1b_profile40_20261012/prepared'
LIB=WS/'runs/e1b_local_profile_20261012/release_2'
EXE='/workspace/cbs_gtsam4.3/devel/lib/glim_ros/glim_rosnode'
PORT=11407
def inside(p): return '/workspace/'+str(p.relative_to('/home/yeranis/repos/V4RL'))
def docker(args,**kw): return subprocess.run(['docker','exec','cbsms_ws_gpu',*args],**kw)
def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()
class ContainerRpc:
    def __init__(self,url): self.url=url
    def __getattr__(self,name):
        def call(*args):
            script='import json,sys,xmlrpc.client; print(json.dumps(getattr(xmlrpc.client.ServerProxy(sys.argv[1]),sys.argv[2])(*json.loads(sys.argv[3]))))'
            r=docker(['python3','-c',script,self.url,name,json.dumps(args)],capture_output=True,text=True)
            if r.returncode: raise OSError(r.stderr)
            return json.loads(r.stdout)
        return call
def main():
    p=argparse.ArgumentParser();p.add_argument('tag');p.add_argument('--library',type=Path,default=LIB)
    p.add_argument('--gdb',choices=['abort','watch']);p.add_argument('--profile',action='store_true')
    p.add_argument('--asan',action='store_true');p.add_argument('--expect-crash',action='store_true')
    p.add_argument('--asan-verbose',action='store_true')
    a=p.parse_args();out=RUN/a.tag;out.mkdir(parents=True,exist_ok=False)
    master=None;node=None;logs=[];pid=None
    setup='source /opt/ros/noetic/setup.bash && source /workspace/cbs_gtsam4.3/devel/setup.bash && '
    env='export ROS_MASTER_URI=http://127.0.0.1:'+str(PORT)+' ROS_IP=127.0.0.1 ROS_LOG_DIR='+inside(out/'ros_logs')+' && '
    def start(label,cmd):
        f=(out/(label+'.log')).open('x');logs.append(f)
        return subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT)
    def stop(name):
        script='import os,signal,sys; from pathlib import Path; p=Path(sys.argv[1]); pid=int(p.read_text()) if p.exists() else 0; c=Path("/proc/%s/cmdline"%pid); ok=pid>1 and c.exists() and sys.argv[2].encode() in c.read_bytes(); (os.killpg(os.getpgid(pid),signal.SIGCONT),os.killpg(os.getpgid(pid),signal.SIGINT)) if ok else None'
        docker(['python3','-c',script,inside(out/(name+'.pid')),str(PORT) if name=='master' else 'glim_rosnode'],check=True)
    try:
        import socket
        ip=subprocess.check_output(['docker','inspect','--format','{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}','cbsms_ws_gpu'],text=True).strip()
        with socket.socket() as s: assert s.connect_ex((ip,PORT))!=0,'occupied test port'
        for path,h in json.loads((FAILED/'protected_manifest.json').read_text()).items(): assert sha(path)==h,path
        cmd=setup+env+'echo $$ > '+inside(out/'master.pid')+'; exec roscore -p '+str(PORT)
        master=start('master',['docker','exec','cbsms_ws_gpu','setsid','--wait','bash','-lc',cmd])
        rpc=ContainerRpc('http://127.0.0.1:'+str(PORT))
        for _ in range(100):
            try:
                if rpc.getPid('/e1b_debug')[0]==1: break
            except OSError: time.sleep(.1)
        else: raise RuntimeError('master not ready')
        params=yaml.safe_load((FAILED/'preflight/resolved_params.yaml').read_text())
        selected={k:v for k,v in params.items() if k.startswith('/glim_rosnode/') or k=='/use_sim_time'}
        # Only output paths change. Config points at the already-copied frozen tree.
        for k,v in selected.items():
            if isinstance(v,str) and '/pilot_on_source/' in v:
                if k.endswith('/config_path'): selected[k]=inside(FAILED/'profile40_on_source/glim_config')
                else: selected[k]=inside(out/k.rsplit('/',1)[-1])
            assert rpc.setParam('/e1b_debug',k,selected[k])[0]==1
        (out/'params.json').write_text(json.dumps(selected,indent=2))
        # Seed clock once. No bag, sensor publisher, Kimera or receiver exists.
        docker(['bash','-lc',setup+env+"rostopic pub -1 /clock rosgraph_msgs/Clock '{clock: {secs: 1727775007, nsecs: 0}}'"],stdout=subprocess.DEVNULL,check=True)
        extra='export LD_LIBRARY_PATH='+inside(a.library)+':/workspace/cbs_gtsam4.3/build/kimera_vio:$LD_LIBRARY_PATH && '
        if a.profile: extra+='export CBS_E1B_PROFILE_PATH='+inside(out/'profile.jsonl')+' && '
        else: extra+='unset CBS_E1B_PROFILE_PATH && '
        if a.gdb: extra+='export LD_PRELOAD='+inside(RUN/'pause.so')+' && '
        if a.asan:
            options='detect_leaks=0:halt_on_error=1'
            if a.asan_verbose: options+=':verbosity=1:log_path='+inside(out/'asan_runtime')
            extra+='export LD_PRELOAD=/usr/lib/gcc/x86_64-linux-gnu/9/libasan.so ASAN_OPTIONS='+options+' && '
        cmd=setup+env+extra+'echo $$ > '+inside(out/'node.pid')+'; exec '+EXE+' __name:=glim_rosnode /os_cloud_node/imu:=/boxi/cpt7/imu /os_cloud_node/points:=/boxi/hesai/points'
        (out/'command.txt').write_text(cmd+'\n')
        node=start('node',['docker','exec','cbsms_ws_gpu','setsid','--wait','bash','-lc',cmd])
        if a.gdb:
            for _ in range(100):
                hits=[]
                for proc in Path('/proc').iterdir():
                    if not proc.name.isdigit(): continue
                    try:
                        argv=(proc/'cmdline').read_bytes().split(b'\0')
                        if argv[0].decode()==EXE and inside(out).encode() in (proc/'environ').read_bytes(): hits.append(int(proc.name))
                    except OSError: pass
                if len(hits)==1 and 'T' in Path('/proc/%s/status'%hits[0]).read_text().split('State:',1)[1].splitlines()[0]:
                    pid=hits[0];break
                time.sleep(.1)
            assert pid,'stopped debug target missing'
            script=['set pagination off','set confirm off','set debuginfod enabled off',
                    'set sysroot /proc/%d/root'%pid,'attach %d'%pid,'handle SIGSTOP nostop noprint pass',
                    'info sharedlibrary']
            if a.gdb=='watch':
                script+=['break glim::GlimCbsBridge::GlimCbsBridge(ros::NodeHandle&)',
                         'continue','printf "BRIDGE_THIS=%p\\n", $rdi','set $bridge = $rdi',
                         'watch -l *(unsigned long*)($bridge+3296)','continue','bt 12','x/8i $pc-12']
            else: script+=['continue','bt 20','info registers']
            script+=['kill','quit']
            (out/'gdb.txt').write_text('\n'.join(script)+'\n')
            with (out/'gdb.log').open('x') as f:
                subprocess.run(['gdb','-nx','-batch','-x',str(out/'gdb.txt')],stdout=f,stderr=subprocess.STDOUT,timeout=40,check=True)
            node.wait(timeout=10)
        elif a.expect_crash:
            assert node.wait(timeout=30)!=0
            if a.asan: assert 'AddressSanitizer: heap-buffer-overflow' in (out/'node.log').read_text()
        else:
            docker(['bash','-lc',setup+env+"rostopic pub -1 /clock rosgraph_msgs/Clock '{clock: {secs: 1727775007, nsecs: 0}}'"],stdout=subprocess.DEVNULL,check=True)
            deadline=time.monotonic()+30
            while time.monotonic()<deadline:
                assert node.poll() is None,'startup exited'
                try:
                    lookup=rpc.lookupNode('/e1b_debug','/glim_rosnode')
                    if lookup[0]==1:
                        api=ContainerRpc(lookup[2]);subs=api.getSubscriptions('/e1b_debug')[2]
                        if any(x[0]=='/boxi/hesai/points' for x in subs): break
                except OSError: pass
                time.sleep(.2)
            else: raise RuntimeError('GLIM readiness timeout')
            cpid=api.getPid('/e1b_debug')[2]
            maps=docker(['cat','/proc/%s/maps'%cpid],capture_output=True,text=True,check=True).stdout
            (out/'maps.txt').write_text(maps)
            assert inside(a.library/'libglim_ros.so') in maps
            time.sleep(2)
            assert node.poll() is None
            state=rpc.getSystemState('/e1b_debug')[2]
            assert not any(topic in ['/boxi/cpt7/imu','/boxi/hesai/points','/kimera/cbs/odom_belief_out'] and pubs for topic,pubs in state[0])
            (out/'ready.json').write_text(json.dumps(dict(passed=True,subscriptions=subs,source_measurements=0,profile_enabled=a.profile,ros_system_state=state),indent=2))
    finally:
        if node and node.poll() is None:
            stop('node');node.wait(timeout=20)
        if master and master.poll() is None: stop('master');master.wait(timeout=20)
        for f in logs: f.close()
    if not a.gdb and not a.expect_crash:
        assert node.returncode==0,('shutdown',node.returncode)
        text=(out/'node.log').read_text()
        for error in ['ERROR: AddressSanitizer','E1B_PROFILE_SINK_ERROR','malloc.c:','terminate called']:
            assert error not in text,error
    (out/'result.json').write_text(json.dumps(dict(node_returncode=node.poll(),master_returncode=master.poll(),debug=a.gdb,expected_crash=a.expect_crash,passed=not bool(a.gdb)),indent=2))
    print(out)
if __name__=='__main__':main()
