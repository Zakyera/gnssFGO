"""Owned finite build/test stage with container restoration and no sensor play."""
import datetime,hashlib,json,subprocess,sys,time
from pathlib import Path
H=Path(__file__).resolve().parent
R=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs')/H.name
P=R.parent/'eig2_persistent_live_cpv80_20261101/prepared'
container='cbsms_ws_gpu'
def call(args):return subprocess.check_output(args,text=True)
def sha(path):
    with Path(path).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def save(path,data):
    with path.open('x') as f:json.dump(data,f,indent=2)
def main():
    mode,attempt=sys.argv[1:];receipt=H/(mode+'_'+attempt);receipt.mkdir(exist_ok=False)
    before=call(['docker','inspect','-f','{{.State.Status}}',container]).strip();assert before=='exited'
    pins=json.loads((P/'protected_manifest.json').read_text())
    assert all(sha(path)==want for path,want in pins.items())
    save(receipt/'start.json',dict(started=datetime.datetime.now().astimezone().isoformat(),before=before,
        protected_count=len(pins),inputs={str(p):sha(p) for p in R.iterdir() if p.is_file()}))
    code=None
    try:
        subprocess.run(['docker','start',container],check=True)
        command=['docker','exec',container,'bash','-lc',
            'source /opt/ros/noetic/setup.bash && python3 /workspace/cbs_gtsam4.3/runs/'+H.name+'/build.py '+mode+' '+attempt]
        with (receipt/'driver.log').open('x') as log:
            job=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT)
            save(receipt/'process.json',dict(pid=job.pid,command=command,proc_start=Path(f'/proc/{job.pid}/stat').read_text().split()[21]))
            code=job.wait()
    finally:
        top=call(['docker','top',container,'-eo','pid,comm,args'])
        save(receipt/'top_before_stop.json',dict(text=top))
        allowed={'tail','zsh'}
        extra=[line for line in top.splitlines()[1:] if line.strip() and line.split()[1] not in allowed]
        assert not extra,('unexpected jobs; do not stop container',extra)
        subprocess.run(['docker','stop',container],check=True)
        changed=[path for path,want in pins.items() if sha(path)!=want]
        save(receipt/'closeout.json',dict(code=code,containers=call(['docker','inspect','-f','{{.Name}} {{.State.Status}}',container,'ubuntu-jammy-humble-nvidia']).splitlines(),protected_changed=changed))
        assert not changed
    assert code==0,('build/test failed; evidence retained',receipt)
if __name__=='__main__':main()
