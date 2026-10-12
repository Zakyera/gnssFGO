"""Replay logged Kimera tracking transactions, including nested recovery."""
import re

def check_kimera(lines):
    active={};tracked=removed=0;removals=[];rollbacks=[];checkpoint=None
    staged_guard=False;active_guards={};pending_active_commits=set();reason=None
    def snapshot():return active.copy(),tracked,removed,len(removals)
    for ln,line in enumerate(lines,1):
        if 'CBS_BPSAM_ACTIVE_WINDOW_LIFECYCLE_ROW,k,' in line:
            assert not pending_active_commits and reason is None, ('unfinished retry',ln)
            fields=line.split('CBS_BPSAM_ACTIVE_WINDOW_LIFECYCLE_ROW,',1)[1].strip().split(',')
            assert int(fields[-2])==len(active),(ln,fields,len(active))
            checkpoint=snapshot()
        if 'action=rollback_and_retry_without_staged_cbs' in line:
            assert not staged_guard;staged_guard=True
        if 'action=rollback_remove_active_and_retry' in line:
            m=re.search(r'k,g,(x\d+->x\d+),.*slot=(\d+),slot_valid=true',line)
            assert m,(ln,'missing valid slot');edge,slot=m.groups()
            assert active.get(edge)==int(slot) and edge not in active_guards,(ln,edge,slot)
            active_guards[edge]=int(slot)
        if ',active_window_rollback,' in line:
            assert checkpoint is not None and reason is None
            reason=line.split('reason=',1)[1].strip()
            assert reason in ['staged_residual_regression','active_residual_regression'],(ln,reason)
            assert staged_guard if reason=='staged_residual_regression' else bool(active_guards)
            previous,nt,nr,nrows=checkpoint
            rollbacks.append(dict(line=ln,reason=reason,undone_insertions=tracked-nt,
                                  undone_removals=removed-nr,restored_active=len(previous)))
            active=previous.copy();tracked=nt;removed=nr;del removals[nrows:]
            # Guard slots originate AFTER the rejected solve. They must still
            # refer to the same factor in the restored graph before removal.
            if reason=='active_residual_regression':
                assert all(active.get(e)==s for e,s in active_guards.items()),('stale restored slot',ln)
                pending_active_commits=set(active_guards)
            staged_guard=False
        m=re.search(r'CBS_BPSAM_ACTIVE_WINDOW_RECEIVER_EDGE_ROW,k,g,(x\d+->x\d+),(active_window_slot_tracked|active_window_remove_scheduled|active_window_remove_committed),slot=(\d+),(.*)',line)
        if m:
            edge,action,slot,tail=m.groups();slot=int(slot)
            total=int(re.search(r'total_active=(\d+)',tail)[1])
            if action=='active_window_slot_tracked':
                assert edge not in active,(ln,edge)
                assert slot not in active.values(),('duplicate live factor slot',ln,slot)
                active[edge]=slot;tracked+=1
            else:
                assert active.pop(edge)==slot,(ln,edge,slot)
                if action=='active_window_remove_scheduled':
                    assert 'reason=update_remove_factor,slot_valid=true' in tail
                else:
                    assert reason=='active_residual_regression' and edge in pending_active_commits
                    assert 'reason=active_residual_regression,slot_valid=true' in tail
                    pending_active_commits.remove(edge)
                removed+=1;removals.append(dict(line=ln,edge=edge,slot=slot,action=action))
            assert total==len(active),(ln,total,len(active))
        if ',active_window_retry_local_update_' in line:
            assert reason is not None and not pending_active_commits
            assert int(re.search(r'preserved_active=(\d+)',line)[1])==len(active)
            if 'without_staged_cbs,' in line:assert reason=='staged_residual_regression'
            else:
                assert 'removing_active_cbs,' in line and reason=='active_residual_regression'
                for field in ['active_removal_count','removal_slot_count','committed_removals']:
                    assert int(re.search(field+r'=(\d+)',line)[1])==len(active_guards)
            # Recursive BPSAM::update creates a NEW backup here, not another
            # IFLS lifecycle line. Later rollback must restore this checkpoint.
            checkpoint=snapshot();reason=None;active_guards={}
    assert reason is None and not staged_guard and not active_guards and not pending_active_commits
    assert tracked and removed and tracked-removed==len(active)
    return dict(tracked=tracked,removed_by_update=removed,remaining=len(active),
                rollbacks=rollbacks,removals=removals,restored_guard_slots_validated=True)
