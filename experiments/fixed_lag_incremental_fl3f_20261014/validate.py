"""Read-only validity and predeclared fidelity checks; no estimator rerun."""
import csv,fcntl,importlib.util,json,math,sys
from pathlib import Path
import numpy as np
from driver import ROOT,REFERENCE,HERE,sha,protect,check_patch

PREFIX=Path('/home/yeranis/results/gnssfgo_fixed_lag_isam_convergence_fl3d_20261013/archive')
LIMITS={'p_m':.01,'orientation_rad':math.radians(.1),'v_mps':.01,'w_radps':.01}
def lines(path):return list(csv.reader(path.open()))
def matrix(path):
    shape=tuple(map(int,path.open().readline().split()))
    a=np.loadtxt(path,skiprows=1).reshape(shape)
    assert np.isfinite(a).all()
    return a

def compare_prior(ar,br,ac,bc):
    assert ar.shape[1]==ac.shape[1] and np.isfinite(np.r_[ar.ravel(),br.ravel(),ac.ravel(),bc.ravel()]).all()
    hr=ar.T@ar;hc=ac.T@ac;gr=-(ar.T@br).ravel();gc=-(ac.T@bc).ravel()
    er,ur=np.linalg.eigh(hr);ec=np.linalg.eigvalsh(hc)
    sr=max(float(np.max(np.abs(er))),1e-300);sc=max(float(np.max(np.abs(ec))),1e-300)
    positive=er>1e-12*sr;pc=ec>1e-12*sc
    w=ur[:,positive]/np.sqrt(er[positive])[None,:]
    delta=w.T@(hc-hr)@w;delta=(delta+delta.T)/2
    info=float(np.max(np.abs(np.linalg.eigvalsh(delta)))) if w.shape[1] else 0.
    score=w.T@(gc-gr);rms=float(np.sqrt(np.mean(score**2))) if score.size else 0.
    null=ur[:,~positive]
    leak=float(np.linalg.norm(hc@null,2)/sc) if null.shape[1] else 0.
    result=dict(dimension=hr.shape[0],reference_rank=int(positive.sum()),candidate_rank=int(pc.sum()),
      reference_eigen_min=float(er[0]),candidate_eigen_min=float(ec[0]),
      max_relative_information_change=info,normalized_gradient_rms_change=rms,
      relative_nullspace_leakage=leak,
      rank_equal=bool(positive.sum()==pc.sum()),psd=bool(er[0]>=-1e-12*sr and ec[0]>=-1e-12*sc))
    result['passed']=bool(result['rank_equal'] and result['psd'] and info<=.05 and rms<=.1 and leak<=1e-10)
    return result

def selfcheck():
    a=np.diag([1.,2.,0.]);b=np.zeros((3,1))
    same=compare_prior(a,b,a,b);assert same['passed'] and same['max_relative_information_change']==0
    scaled=compare_prior(a,b,math.sqrt(1.1)*a,b)
    assert not scaled['passed'] and abs(scaled['max_relative_information_change']-.1)<1e-14
    shifted=compare_prior(a,b,a,np.array([[.2],[.2],[0.]]))
    assert not shifted['passed'] and abs(shifted['normalized_gradient_rms_change']-.2)<1e-14
    rank=compare_prior(a,b,np.diag([1.,2.,1.]),b);assert not rank['passed'] and not rank['rank_equal']
    # A pure rotation of the informed plane preserves rank, but leaks into its nullspace.
    angle=.001;q=np.array([[math.cos(angle),0,-math.sin(angle)],[0,1,0],[math.sin(angle),0,math.cos(angle)]])
    leak=compare_prior(a,b,a@q,b);assert leak['rank_equal'] and not leak['passed'] and leak['relative_nullspace_leakage']>1e-10
    return dict(identity=True,information_change=True,gradient_change=True,rank_change=True,nullspace_change=True)

def main():
    checked=selfcheck()
    if sys.argv[1:]==['--selfcheck']:print(json.dumps(checked));return
    with (ROOT/'controller.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        assert not (ROOT/'validation.json').exists()
        check_patch();saved=json.loads((ROOT/'protected.json').read_text());assert saved==protect()
        run=json.loads((ROOT/'observe.json').read_text())
        assert run['returncode'] in (0,10) and run['protected_unchanged']
        d=ROOT/'diagnostics';old=REFERENCE/'diagnostics'
        result=dict(line.split(maxsplit=1) for line in (d/'result.txt').read_text().splitlines())
        success=result['published']=='1';assert (run['returncode']==0)==success
        spec=importlib.util.spec_from_file_location('fl3d_checks',HERE.parent/'fixed_lag_isam_convergence_fl3d_20261013/validate.py')
        helper=importlib.util.module_from_spec(spec);spec.loader.exec_module(helper)
        assert result['reference_pre_steps']=='116' and result['reference_post_steps']=='7'
        assert sha(d/'reference_pre.values')==sha(old/'pre/last_accepted.values')
        assert sha(d/'reference_final.values')==sha(old/'after_event.values')
        assert sha(d/'pre/initial.values')==sha(old/'pre/initial.values')
        previous={str(p.relative_to(PREFIX)) for p in PREFIX.rglob('*') if p.is_file()}
        archive=ROOT/'archive';current={str(p.relative_to(archive)) for p in archive.rglob('*') if p.is_file()}
        assert previous<=current
        for name in sorted(previous):assert sha(PREFIX/name)==sha(archive/name),('prefix changed',name)
        assert not list(archive.glob('.stage_*'));extra=current-previous
        for prefix in ('before_','after_'):
            n=lambda name:int(result[prefix+name])
            assert n('inserted')==n('active')+n('absorbed')
            assert n('received')==sum(n(k) for k in ('inserted','replaced','duplicate','stale','excluded','unsupported_end','pending','buffered'))
        before={tuple(r[:4]):r for r in lines(d/'before_ledger.csv')}
        after={tuple(r[:4]):r for r in lines(d/'after_ledger.csv')}
        for key in before.keys()&after.keys():assert before[key]==after[key],('source mean/covariance/slot changed',key)
        comparisons={};fidelity=False
        for phase in ('pre','post'):
            if not (d/phase).exists():continue
            rows=list(csv.DictReader((d/phase/'trials.csv').open()));assert len(rows)<=1
            for row in rows:
                oldcost=float(row['old_cost']);cost=float(row['trial_cost'])
                accepted=math.isfinite(cost) and cost<=oldcost+1e-12*(1+abs(oldcost))
                assert (row['accepted']=='1')==accepted
            assert (result[phase+'_converged']=='1')==(float(result[phase+'_gradient_after'])<=1e-5)
        if success:
            assert result['after_event']=='84958' and result['after_newest']=='1224'
            assert result['after_oldest']=='1154' and result['after_supports']=='71'
            assert result['retirement_calls']==result['reload_calls']=='1'
            assert result['pre_numerically_valid']==result['post_numerically_valid']=='1'
            total=int(result['pre_steps'])+2+int(result['post_steps'])
            assert total<=4 and int(result['post_budget'])==4-int(result['pre_steps'])-2
            assert math.isfinite(float(result['committed_gradient']))
            target=lines(archive/'commits/84958/records.txt')
            pubs=[r for r in target if r[0]=='publication_candidate'];outputs=[r for r in target if r[0]=='incremental_candidate']
            finalized=[r for r in target if r[0]=='finalized'];absorbed=[r for r in target if r[0]=='absorbed']
            assert len(pubs)==len(outputs)==len(finalized)==1
            assert not any(r[0] in ('publication','online') for r in target)
            assert pubs[0][1]=='1224' and pubs[0][4]==str(total) and pubs[0][5]=='1154'
            assert float(pubs[0][3])==float(result['post_gradient_after'])
            assert outputs[0][1]=='1224' and finalized[0][1]=='1153'
            for key,delta in [('publications',1),('finalized',1),('window_rebuilds',1),('retired_blocks',3)]:
                assert int(result['after_'+key])==int(result['before_'+key])+delta
            assert set(before)-set(after)=={tuple(r[1:5]) for r in absorbed}
            expected={'commits/84958/records.txt'}
            for r in absorbed:
                key=tuple(r[1:5]);assert before[key][4]==r[5] and before[key][9]==r[6]
                marker=f'ids/{r[1]}/{r[2]}/{r[3]}_{r[4]}'
                assert (archive/marker).read_text()=='84958\n';expected.add(marker)
            assert extra==expected and max(int(p.name) for p in (archive/'commits').iterdir())==84958
            added=set(after)-set(before);arrived=[r for r in target if r[:2]==['belief','insert']]
            assert added=={tuple(r[2:6]) for r in arrived}
            for phase,a,b in [('pre','reference_pre.values','pre/last_accepted.values'),('post','reference_final.values','after_event.values')]:
                u=helper.values(d/a);v=helper.values(d/b);assert u.keys()==v.keys()
                comparisons[phase]=helper.changes(u,v)
            final=helper.values(d/'after_event.values')
            assert len(final)==213 and min(i for k,i in final)==1154 and max(i for k,i in final)==1224
            result['state_fidelity_passed']=all(stats[k]['maximum']<=limit for stats in comparisons.values() for k,limit in LIMITS.items())
            result['joint_marginal_comparison']=compare_prior(matrix(d/'reference_prior.A'),matrix(d/'reference_prior.b'),matrix(d/'candidate_prior.A'),matrix(d/'candidate_prior.b'))
            fidelity=result['state_fidelity_passed'] and result['joint_marginal_comparison']['passed']
            result.update(total_steps=total,absorbed_this_update=len(absorbed),inserted_pending_this_update=len(added))
        else:
            assert result['rollback_verified']=='1' and not extra
            assert sha(d/'before_ledger.csv')==sha(d/'after_ledger.csv') and sha(d/'before_event.values')==sha(d/'after_event.values')
        result.update(diagnostic_valid=True,structural_candidate_passed=success,reference_byte_identical=True,
          state_comparisons=comparisons,state_limits=LIMITS,fidelity_passed=fidelity,
          classification='CANDIDATE_PASSES_LOCAL_SCREEN' if success and fidelity else 'CANDIDATE_REJECTED',
          prior_archive_files_byte_identical=len(previous),new_archive_files=len(extra),protected_files=len(saved),
          binary_sha256=run['binary_sha256'],max_rss_kib=run['max_rss_kib'],selfchecks=checked,
          production_promoted=False,full_replay_validated=False)
        assert saved==protect()
        with (ROOT/'validation.json').open('x') as f:json.dump(result,f,indent=2,allow_nan=False)
        print(json.dumps(result,indent=2,allow_nan=False))

if __name__=='__main__':main()
