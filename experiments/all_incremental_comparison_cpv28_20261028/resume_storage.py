"""Continue only nine never-started cases after an identified storage guard."""
import os,json,shutil,subprocess,tempfile,time,traceback
from pathlib import Path
import replay as m
H=m.H
def main():
    m.save(H/'storage_resume_start.json',dict(pid=os.getpid(),time=time.time()))
    assert 'SSD free <8GiB' in (H/'replay_failed.txt').read_text()
    assert not (H/'replay_complete.json').exists()
    oldroot=Path(json.loads((H/'scratch.json').read_text())['path']);oldbundle=oldroot/'evidence'
    pins=json.loads((H/'replay_input_pins.json').read_text())
    for p,v in pins.items():assert m.sha(p)==v,('input changed',p)
    for p in (Path(__file__),H/'STORAGE_CONTINUATION.txt'):pins[str(p)]=m.sha(p)
    records=[];jobs=[]
    for case in m.CASES:
        old=json.loads((m.H10/(case+'.json')).read_text())
        for mode in ('off','on'):
            name=case+'_'+mode
            if (H/(name+'_archive.json')).exists():
                receipt=json.loads((H/(name+'_archive.json')).read_text())
                assert json.loads((H/(name+'_parity.json')).read_text())['pass_']
                assert m.sha(oldbundle/(name+'.tar.gz'))==receipt['sha256']
                records.append(dict(case=case,mode=mode,archive=receipt,publication_sha256=m.sha(oldbundle/(name+'_publications.csv'))))
            else:
                assert not (H/(name+'_launch.json')).exists(),'never replay an already launched case'
                jobs.append((case,mode,Path(old['input_products'][mode+'_fusion_online']['path']).parent.parent))
    assert len(records)==3 and len(jobs)==9 and jobs[0][:2]==('hesai_eig2','on')
    scratch=Path(tempfile.mkdtemp(prefix='cpv28-remaining-',dir='/run/user/1000'));bundle=scratch/'evidence';bundle.mkdir()
    m.save(H/'storage_resume_scratch.json',dict(path=str(scratch),previous=str(oldroot),remaining=[(c,mo) for c,mo,_ in jobs]))
    env=dict(os.environ)
    for k in list(env):
        if k.startswith(('WNOJ_','E2_','E2B_','S5B_','S5C_','E4_','CACHE_FAILURE')) or k=='LD_PRELOAD':env.pop(k)
    env.update(LD_LIBRARY_PATH=m.LIB,OMP_NUM_THREADS='1',E2B_ENDPOINT_CACHE='0',E2_EXACT_LINEARIZATION_CACHE='0')
    for index,(case,mode,root) in enumerate(jobs):
        retained=sum(p.stat().st_size for d in (oldbundle,bundle) for p in d.iterdir())
        projected=retained+(len(jobs)-index)*220*2**20
        assert shutil.disk_usage(H).free-projected>4*2**30,'projected SSD headroom <4GiB'
        assert shutil.disk_usage(scratch).free>2.75*2**30,'tmpfs headroom <2.75GiB'
        name=case+'_'+mode;out=scratch/name;pub=bundle/(name+'_publications.csv')
        env['CPV23_PUBLICATION_FILE']=str(pub)
        cmd=['taskset','-c','2',str(m.EXE),'--absorbed-revision-policy','seal_at_absorption','--mode','sources',
             '--timeline','record','--solver','isam2','--support-policy','uniform_imu','--support-period-ns','100000000',
             '--receiver-startup-samples','59','--receiver-model','source_wnoa','--event-stream',str(root/'events.csv'),'--out',str(out)]
        m.status('replay',case=case,mode=mode,continuation='storage_only');t=time.time()
        with (H/(name+'_replay.log')).open('xb') as log:
            p=subprocess.Popen(cmd,env=env,stdout=log,stderr=subprocess.STDOUT)
            m.save(H/(name+'_launch.json'),dict(pid=p.pid,command=cmd,time=t));code=p.wait()
        m.save(H/(name+'_finish.json'),dict(returncode=code,seconds=time.time()-t));assert code==0,name
        m.status('parity',case=case,mode=mode)
        checks=m.parity(root/'receiver',out,pub);m.save(H/(name+'_parity.json'),checks)
        m.status('archive',case=case,mode=mode)
        receipt=m.archive(out,bundle/(name+'.tar.gz'));m.save(H/(name+'_archive.json'),receipt)
        assert out.parent==scratch and out.name==name and not out.is_symlink();shutil.rmtree(out)
        records.append(dict(case=case,mode=mode,archive=receipt,publication_sha256=m.sha(pub)))
        m.status('case_complete',case=case,mode=mode,archive_bytes=receipt['bytes'])
    for p,v in pins.items():assert m.sha(p)==v,('input changed',p)
    needed=sum(p.stat().st_size for d in (oldbundle,bundle) for p in d.iterdir())
    assert shutil.disk_usage(H).free-needed>4*2**30
    dest=H/'replay_evidence';dest.mkdir()
    for srcdir in (oldbundle,bundle):
        for src in srcdir.iterdir():
            target=dest/src.name
            with src.open('rb') as fi,target.open('xb') as fo:shutil.copyfileobj(fi,fo)
            assert m.sha(target)==m.sha(src)
    m.save(H/'replay_complete.json',dict(pass_=True,time=time.time(),records=records,pins=len(pins),persistent_bytes=needed,
         initial_completed_cases=3,continued_unstarted_cases=9,duplicate_replays=0,all_archive_members_verified=True,
         storage_correction=str(H/'STORAGE_CONTINUATION.txt')))
    for path,prefix in ((oldroot,'cpv28-method2-'),(scratch,'cpv28-remaining-')):
        assert path.parent==Path('/run/user/1000') and path.name.startswith(prefix);shutil.rmtree(path)
    m.status('complete')
if __name__=='__main__':
    try:main()
    except Exception:
        with (H/'storage_resume_failed.txt').open('x') as f:f.write(traceback.format_exc())
        raise
