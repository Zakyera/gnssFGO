"""Read-only validation/state comparison of the single diagnostic run."""
import csv,fcntl,json,math
import numpy as np
from scipy.spatial.transform import Rotation
from driver import ROOT,REFERENCE,sha,protect

def values(path):
    result={}
    for line in path.read_text().splitlines():
        kind,index,*data=line.split();a=np.array(data,dtype=float)
        assert np.isfinite(a).all()
        result[(kind,int(index))]=a.reshape(3,4) if kind=='x' else a
    return result

def localmax(a,b):
    assert a.keys()==b.keys();result=0.
    for key,u in a.items():
        v=b[key]
        if np.array_equal(u,v):continue
        if key[0]!='x':d=v-u
        else:
            phi=Rotation.from_matrix(u[:,:3].T@v[:,:3]).as_rotvec()
            t=u[:,:3].T@(v[:,3]-u[:,3]);theta=np.linalg.norm(phi)
            h=np.array([[0,-phi[2],phi[1]],[phi[2],0,-phi[0]],[-phi[1],phi[0],0]])
            c=1/12+theta**2/720 if theta<1e-5 else (1-theta/(2*np.tan(theta/2)))/theta**2
            d=np.r_[phi,(np.eye(3)-h/2+c*h@h)@t]
        result=max(result,float(np.max(np.abs(d))))
    return result

def changes(a,b):
    assert a.keys()==b.keys();groups=dict(p_m=[],orientation_rad=[],v_mps=[],w_radps=[])
    newest=max(i for k,i in a if k=='x');latest={}
    for key,u in a.items():
        kind,index=key;v=b[key]
        if kind=='x':
            d={'p_m':float(np.linalg.norm(v[:,3]-u[:,3])),
               'orientation_rad':float(np.linalg.norm(Rotation.from_matrix(u[:,:3].T@v[:,:3]).as_rotvec()))}
        else:d={('v_mps' if kind=='v' else 'w_radps'):float(np.linalg.norm(v-u))}
        for name,x in d.items():groups[name].append(x)
        if index==newest:latest.update(d)
    return {name:dict(rms=float(np.sqrt(np.mean(np.square(xs)))),maximum=max(xs),
                      newest=latest[name]) for name,xs in groups.items()}

def main():
    with (ROOT/'controller.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        assert not (ROOT/'validation.json').exists()
        run=json.loads((ROOT/'observe.json').read_text())
        assert run['returncode']==0 and run['protected_unchanged']
        saved=json.loads((ROOT/'protected.json').read_text());assert saved==protect()
        d=ROOT/'diagnostics';snap=REFERENCE/'diagnostics/snapshot'
        result=dict(line.split(maxsplit=1) for line in (d/'result.txt').read_text().splitlines())
        rows=list(csv.DictReader((d/'trials.csv').open()))
        old=list(csv.DictReader((REFERENCE/'diagnostics/trials.csv').open()))
        assert len(rows)==int(result['attempts'])<=200
        assert result['rollback_event']=='84957' and result['rollback_newest']=='1223'
        assert result['requested_budget']=='20' and result['variables']=='216' and result['fixed_keys']=='0'
        assert len(rows)>=20,'target stopped before original20steps'
        comparisons=[]
        for i in range(21):
            name='initial' if i==0 else 'trial_'+str(i)
            state_error=localmax(values(snap/(name+'.values')),values(d/(name+'.values')))
            cost=float(result['initial_cost']) if i==0 else float(rows[i-1]['trial_cost'])
            gradient=float(result['initial_gradient']) if i==0 else float(rows[i-1]['accepted_gradient'])
            cost_error=abs(cost-float(old[i]['cost']));gradient_error=abs(gradient-float(old[i]['gradient']))
            comparisons.append(dict(step=i,max_local_difference=state_error,cost_difference=cost_error,
              gradient_difference=gradient_error,byte_identical=sha(snap/(name+'.values'))==sha(d/(name+'.values'))))
        reference_pass=all(x['max_local_difference']<=1e-8 and x['cost_difference']<=1e-7
                           and x['gradient_difference']<=1e-7 for x in comparisons)
        # Prove the full pre-target state/archival history was restored, not just a saved batch objective.
        oldroot=REFERENCE/'archive';newroot=ROOT/'archive'
        oldfiles={str(p.relative_to(oldroot)) for p in oldroot.rglob('*') if p.is_file()}
        newfiles={str(p.relative_to(newroot)) for p in newroot.rglob('*') if p.is_file()}
        assert oldfiles==newfiles,'prior archive file sets differ'
        for name in sorted(oldfiles):assert sha(oldroot/name)==sha(newroot/name),('prior archive differs',name)
        assert not list(newroot.glob('.stage_*'))
        assert max(int(p.name) for p in (newroot/'commits').iterdir())==84956
        accepted=[r for r in rows if r['accepted']=='1']
        for r in rows:
            finite=math.isfinite(float(r['trial_cost']))
            expected=finite and float(r['trial_cost'])<=float(r['old_cost'])+1e-12*(1+abs(float(r['old_cost'])))
            assert (r['accepted']=='1')==expected
        passed=math.isfinite(float(result['final_accepted_gradient'])) and float(result['final_accepted_gradient'])<=1e-5
        assert (result['passed']=='1')==passed
        final=values(d/'last_accepted.values');initial=values(d/'initial.values')
        result.update(reference_pass=reference_pass,first20_comparisons=comparisons,
          prior_archive_files_byte_identical=len(oldfiles),protected_files=len(saved),
          accepted_steps=len(accepted),rejected_steps=len(rows)-len(accepted),
          initial_to_final=changes(initial,final),
          step20_to_final=changes(values(d/'trial_20.values'),final),
          last_accepted_step_change=changes(values(d/('trial_'+str(len(accepted)-1)+'.values')),final),
          update_seconds=sum(float(r['update_seconds']) for r in rows),
          gradient_check_seconds=sum(float(r['check_seconds']) for r in rows),
          first20_seconds=sum(float(r['total_seconds']) for r in rows[:20]),
          beyond20_seconds=sum(float(r['total_seconds']) for r in rows[20:]),
          maximum_step_seconds=max(float(r['total_seconds']) for r in rows),
          max_rss_kib=run['max_rss_kib'],binary_sha256=run['binary_sha256'],
          diagnostic_valid=reference_pass,production_gate_passed=False)
        assert saved==protect()
        with (ROOT/'validation.json').open('x') as f:json.dump(result,f,indent=2)
        print(json.dumps({k:v for k,v in result.items() if k!='first20_comparisons'},indent=2))
        assert reference_pass,'actual incremental reference-path check failed; stop without interpretation'

if __name__=='__main__':main()
