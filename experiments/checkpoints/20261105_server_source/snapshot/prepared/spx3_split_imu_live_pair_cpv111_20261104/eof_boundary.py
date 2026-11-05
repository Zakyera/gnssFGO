"""Read-only proof of final-cell exclusions, not a receiver implementation."""
import csv
from pathlib import Path

PERIOD=100_000_000

def boundary_condition(origin, newest, oldest, clock, start, end):
    last=origin+newest*PERIOD
    next_support=last+PERIOD
    assert last<=clock<next_support, 'last support/clock mismatch'
    assert origin+oldest*PERIOD<=start<end, 'invalid or retired first endpoint'
    assert last<end<=clock, 'not a measured endpoint in final partial cell'
    assert (end-origin+PERIOD-1)//PERIOD==newest+1, 'more than next support missing'
    return dict(last_support_ns=last,next_required_support_ns=next_support,
        last_clock_ns=clock,from_ns=start,to_ns=end,
        end_after_last_support_ms=(end-last)*1e-6,
        missing_clock_ms=(next_support-clock)*1e-6)

def audit_eof(root,mode,summary,actions,online_records,events):
    root=Path(root)/f'full_{mode}_receiver/receiver'
    origin=int(online_records[0][2]);newest=int(online_records[-1][1])
    last=int(online_records[-1][2]);assert last==origin+newest*PERIOD
    clocks=[int(e[2]) for e in events if e[0]=='I'];assert clocks
    clock=clocks[-1];assert last<=clock<last+PERIOD
    with (root/'stream_progress.csv').open() as f:
        progress=list(csv.DictReader(f))
    end=progress[-1];before=progress[-2]
    assert end['kind']=='E' and int(end['event'])==len(events)
    assert int(end['pending'])==summary['pending_at_end']==0
    assert int(end['newest'])==newest
    # EOF only clears unsupported entries/finalizes the existing solver state.
    for key in ('publications','newest','oldest','active','supports','slots','nonempty'):
        assert before[key]==end[key],('EOF graph shape changed',key)
    with (root/f"archive/commits/{end['event']}/records.txt").open() as f:
        rows=list(csv.reader(f))
    unsupported=[r for r in rows if r[:2]==['belief','unsupported_at_end']]
    assert len(unsupported)==actions['unsupported_at_end']==summary['unsupported_end']
    assert int(before['pending'])==len(unsupported)
    details=[]
    for r in unsupported:
        assert int(r[11])==int(events[-1][1]), 'not an EOF classification'
        assert int(r[9])<=int(r[10])<=int(events[-1][1]), 'noncausal physical/arrival time'
        item=boundary_condition(origin,newest,int(end['oldest']),clock,int(r[8]),int(r[9]))
        item.update(source=r[2],session=r[3],from_id=r[4],to_id=r[5],revision=r[6],
                    source_ordinal=r[7],arrival_ns=int(r[10]),classified_event=int(end['event']))
        details.append(item)
    # No source insertion or replacement occurs at EOF.
    assert all(r[0] in ('belief','finalized_incremental','drain_incremental','end_incremental') for r in rows)
    assert all(r[1]=='unsupported_at_end' for r in rows if r[0]=='belief')
    return dict(passed=True,contract='finite_recording_terminal_cell_only',
        original_zero_unsupported_gate_passed=not details,excluded_versions=len(details),
        final_partial_cell_ns=clock-last,details=details,
        state_clock_not_extended=True,captured_outputs_unchanged=True,
        omission_not_recovered=True)

def test_boundaries():
    assert boundary_condition(0,10,0,1_030_000_000,900_000_000,1_011_000_000)
    bad=[(900_000_000,999_000_000),(900_000_000,1_050_000_000),
         (900_000_000,1_200_000_000),(-1,1_011_000_000)]
    for start,end in bad:
        try:boundary_condition(0,10,0,1_030_000_000,start,end)
        except AssertionError:continue
        raise AssertionError(('invalid terminal exception accepted',start,end))
    return dict(valid=1,rejected=4)
