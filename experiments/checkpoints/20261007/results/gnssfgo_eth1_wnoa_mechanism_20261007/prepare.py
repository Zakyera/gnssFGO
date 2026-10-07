from pathlib import Path
import csv, hashlib, json, subprocess

ROOT = Path(__file__).resolve().parent
RUN = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_eth1_live_wnoa_20261007')
PREVIOUS = Path('/home/yeranis/results/gnssfgo_eth1_local_prediction_20261007')
REPO = Path('/home/yeranis/repos/gnssFGO')
TAGS = {'off':'full_off', 'on':'bounded120_on'}

def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        for b in iter(lambda:f.read(2**20),b''):h.update(b)
    return h.hexdigest()

if __name__ == '__main__':
    assert sha(PREVIOUS/'build/wnoa/replay') == 'fb56ffc5c6c9cb575d6129289688caeabb4e3baba32e429b857f3d6a30e50d1f'
    assert json.loads((RUN/'bounded120_on_validation.json').read_text())['passed']
    end=int((RUN/'bounded120_on_receiver/events.csv').read_text().splitlines()[-1].split(',')[1])
    first=int((RUN/'full_off_receiver/events.csv').open().readline().split(',')[1])
    cutoff=min(first+120000000000,end)
    (ROOT/'inputs').mkdir(exist_ok=False)
    inputs={};protected={}
    for mode,tag in TAGS.items():
        source=RUN/(tag+'_receiver/events.csv');kept=[];counts={};last=0
        for line in source.open():
            row=line.rstrip('\n').split(',');arrival=int(row[1])
            assert arrival>=last;last=arrival
            if arrival<=cutoff and row[0]!='E':
                kept.append(line);counts[row[0]]=counts.get(row[0],0)+1
        with (ROOT/'inputs'/f'{mode}_events.csv').open('x') as f:
            f.writelines(kept);f.write(f'E,{cutoff}\n')
        inputs[mode]={'original':str(source),'counts':counts,'original_sha256':sha(source),'clipped_sha256':sha(ROOT/'inputs'/f'{mode}_events.csv')}
        protected[str(source)]=sha(source)
        for p in (RUN/(tag+'_receiver/receiver')).glob('*'):
            if p.is_file():protected[str(p)]=sha(p)
        for name in ['resolved_glim_params.yaml','resolved_kimera_params.yaml','glim_odometry.csv','kimera_odometry.csv']:
            p=RUN/(tag+'_source')/name;protected[str(p)]=sha(p)
        for folder in ['glim_config','kimera_profile']:
            for p in (RUN/(tag+'_source')/folder).rglob('*'):
                if p.is_file():protected[str(p)]=sha(p)
    for p in (PREVIOUS/'build/wnoa').rglob('*'):
        if p.is_file():protected[str(p)]=sha(p)
    for name in ['tools/score_grandtour_prism.py','tools/score_grandtour_relative_pose.py']:
        p=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/cbsms')/name;protected[str(p)]=sha(p)
    # Physical IMU values/timestamps must match; arrival ordering is retained per run.
    imu=[]
    for mode in TAGS:
        imu.append([r[2:] for r in csv.reader((ROOT/'inputs'/f'{mode}_events.csv').open()) if r[0]=='I'])
    assert imu[0]==imu[1]
    report={'first_imu_ns':first,'cutoff_ns':cutoff,'cutoff_elapsed_s':(cutoff-first)*1e-9,'inputs':inputs,'physical_imu_equal':True,
            'protected':protected,'head':subprocess.check_output(['git','-C',str(REPO),'rev-parse','HEAD'],text=True).strip(),
            'tracked_diff_sha256':hashlib.sha256(subprocess.check_output(['git','-C',str(REPO),'diff','HEAD','--binary'])).hexdigest()}
    with (ROOT/'manifest.json').open('x') as f:json.dump(report,f,indent=2)
    print(json.dumps({k:v for k,v in report.items() if k!='protected'},indent=2))
