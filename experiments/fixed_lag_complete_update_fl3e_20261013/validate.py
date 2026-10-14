"""Validate the one complete target transaction, without reoptimization."""
import csv,fcntl,importlib.util,json
from pathlib import Path
from driver import ROOT,REFERENCE,HERE,sha,protect,check_patch

def lines(path):return list(csv.reader(path.open()))
def main():
    with (ROOT/'controller.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        assert not (ROOT/'validation.json').exists()
        check_patch();saved=json.loads((ROOT/'protected.json').read_text());assert saved==protect()
        run=json.loads((ROOT/'observe.json').read_text())
        assert run['returncode'] in (0,10) and run['protected_unchanged']
        d=ROOT/'diagnostics';old=REFERENCE/'diagnostics'
        result=dict(line.split(maxsplit=1) for line in (d/'result.txt').read_text().splitlines())
        success=result['published']=='1';assert (run['returncode']==0)==success
        # Reuse only read-only Values/SE(3) comparison helpers from FL3d.
        spec=importlib.util.spec_from_file_location('fl3d_checks',HERE.parent/'fixed_lag_isam_convergence_fl3d_20261013/validate.py')
        helper=importlib.util.module_from_spec(spec);spec.loader.exec_module(helper)
        comparisons=[]
        newrows=list(csv.DictReader((d/'pre/trials.csv').open()))
        oldrows=list(csv.DictReader((old/'trials.csv').open()))
        assert len(newrows)==len(oldrows)==116
        for i in range(117):
            name='initial' if i==0 else 'trial_'+str(i)
            a=old/(name+'.values');b=d/'pre'/(name+'.values')
            error=helper.localmax(helper.values(a),helper.values(b))
            differences={k:0. if i==0 else abs(float(newrows[i-1][k])-float(oldrows[i-1][k]))
                         for k in ('old_cost','trial_cost','accepted_gradient')}
            assert error<=1e-8 and max(differences.values())<=1e-7,('pre-solve reproduction',i,error,differences)
            comparisons.append(dict(step=i,byte_identical=sha(a)==sha(b),local_difference=error,**differences))
        assert sha(d/'pre/last_accepted.values')==sha(old/'last_accepted.values')
        oldroot=REFERENCE/'archive';newroot=ROOT/'archive'
        previous={str(p.relative_to(oldroot)) for p in oldroot.rglob('*') if p.is_file()}
        current={str(p.relative_to(newroot)) for p in newroot.rglob('*') if p.is_file()}
        assert previous<=current
        for name in sorted(previous):assert sha(oldroot/name)==sha(newroot/name),('prefix output changed',name)
        assert not list(newroot.glob('.stage_*'))
        extra=current-previous
        for prefix in ('before_','after_'):
            n=lambda name:int(result[prefix+name])
            assert n('inserted')==n('active')+n('absorbed')
            assert n('received')==sum(n(k) for k in ('inserted','replaced','duplicate','stale','excluded','unsupported_end','pending','buffered'))
        before={tuple(r[:4]):r for r in lines(d/'before_ledger.csv')}
        after={tuple(r[:4]):r for r in lines(d/'after_ledger.csv')}
        for key in before.keys()&after.keys():assert before[key]==after[key],('surviving source payload/slot changed',key)
        if success:
            assert result['after_event']=='84958' and result['after_newest']=='1224'
            assert result['after_oldest']=='1154' and result['after_supports']=='71'
            assert result['retirement_calls']==result['reload_calls']=='1'
            assert result['pre_passed']==result['post_passed']=='1'
            total=int(result['pre_steps'])+2+int(result['post_steps'])
            assert total<=200 and int(result['post_budget'])==200-int(result['pre_steps'])-2
            assert float(result['post_gradient_after'])<=1e-5 and float(result['committed_gradient'])<=1e-5
            target=lines(newroot/'commits/84958/records.txt')
            pubs=[r for r in target if r[0]=='publication'];online=[r for r in target if r[0]=='online']
            finalized=[r for r in target if r[0]=='finalized'];absorbed=[r for r in target if r[0]=='absorbed']
            assert len(pubs)==len(online)==len(finalized)==1
            assert pubs[0][1]=='1224' and pubs[0][4]==str(total) and pubs[0][5]=='1154'
            assert float(pubs[0][3])==float(result['post_gradient_after'])
            assert online[0][1]=='1224' and finalized[0][1]=='1153'
            assert int(result['after_publications'])==int(result['before_publications'])+1
            assert int(result['after_finalized'])==int(result['before_finalized'])+1
            assert int(result['after_window_rebuilds'])==int(result['before_window_rebuilds'])+1
            assert int(result['after_retired_blocks'])==int(result['before_retired_blocks'])+3
            assert set(before)-set(after)=={tuple(r[1:5]) for r in absorbed}
            expected={'commits/84958/records.txt'}
            for r in absorbed:
                key=tuple(r[1:5]);assert before[key][4]==r[5] and before[key][9]==r[6]
                marker=f'ids/{r[1]}/{r[2]}/{r[3]}_{r[4]}'
                assert (newroot/marker).read_text()=='84958\n';expected.add(marker)
            assert extra==expected,('unexpected output',extra^expected)
            assert max(int(p.name) for p in (newroot/'commits').iterdir())==84958
            added=set(after)-set(before);arrived=[r for r in target if r[:2]==['belief','insert']]
            assert added=={tuple(r[2:6]) for r in arrived}
            pre=helper.values(d/'pre/last_accepted.values');final=helper.values(d/'after_event.values')
            assert min(i for k,i in final)==1154 and max(i for k,i in final)==1224
            assert len(final)==213
            result['pre_to_final_retained_state_changes']=helper.changes({k:pre[k] for k in final},final)
            result['total_steps']=total;result['absorbed_this_update']=len(absorbed)
            result['inserted_pending_this_update']=len(added)
        else:
            assert result['rollback_verified']=='1' and not extra
            assert sha(d/'before_ledger.csv')==sha(d/'after_ledger.csv')
            assert sha(d/'before_event.values')==sha(d/'after_event.values')
        result.update(diagnostic_valid=True,complete_update_passed=success,
          first116_pre_solve_comparisons=comparisons,prior_archive_files_byte_identical=len(previous),
          new_archive_files=len(extra),protected_files=len(saved),binary_sha256=run['binary_sha256'],
          max_rss_kib=run['max_rss_kib'],full_replay_validated=False)
        assert saved==protect()
        with (ROOT/'validation.json').open('x') as f:json.dump(result,f,indent=2)
        print(json.dumps({k:v for k,v in result.items() if k!='first116_pre_solve_comparisons'},indent=2))

if __name__=='__main__':main()
