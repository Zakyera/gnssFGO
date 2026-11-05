"""Bounded saved-data audit; never invokes a solver, ROS or a container."""
import csv
import hashlib
import importlib.util
import json
from collections import Counter
from itertools import zip_longest
from pathlib import Path
import sys

sys.dont_write_bytecode = True
sys.path.append('/home/yeranis/.local/share/pipx/venvs/evo/lib/python3.12/site-packages')
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent
P = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_spx3_awake_full_cpk26_20261021/prepared')
GEO = HERE.parent/'fixed_lag_eig2_g2k_reference_fl4y_20261017/analyze.py'
spec = importlib.util.spec_from_file_location('checked_se3', GEO)
geo = importlib.util.module_from_spec(spec); spec.loader.exec_module(geo)
BAG = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/datasets/GrandTour/SPX-3/2024-11-02-17-43-10_cpt7_ie_tc.bag')
START = 1730565902872888651
EDGES = ((0, 1), (1, 2), (2, 3))
HASHES = {}


def pin(p):
    HASHES[str(p)] = geo.sha(p)


def dump(p, obj):
    with p.open('x') as f:
        json.dump(obj, f, indent=2, allow_nan=False)


def measure(r, ref):
    C = r.pop('_C'); mu = r.pop('_mu'); Z = geo.exp(mu)
    target = geo.inv(ref.at(r['start_ns'])) @ ref.at(r['end_ns'])
    e = geo.log(geo.inv(Z) @ target)
    T, R = C[3:, 3:], C[:3, :3]
    assert np.isfinite(C).all() and np.max(abs(C-C.T)) < 1e-12
    assert np.linalg.eigvalsh(C).min() > 0
    direction = e[3:]/np.linalg.norm(e[3:])
    r.update(translation_error_mm=float(np.linalg.norm(e[3:])*1000),
             cartesian_error_mm=float(np.linalg.norm(Z[:3,3]-target[:3,3])*1000),
             rotation_error_deg=float(np.linalg.norm(e[:3])*180/np.pi),
             translation_trace_m2=float(np.trace(T)),
             translation_rss_sigma_mm=float(np.sqrt(np.trace(T))*1000),
             rotation_trace_rad2=float(np.trace(R)),
             rotation_rss_sigma_deg=float(np.sqrt(np.trace(R))*180/np.pi),
             translation_error_direction_sigma_mm=float(np.sqrt(direction@T@direction)*1000),
             reference_whitened_norm_6d=float(np.sqrt(e@np.linalg.solve(C,e))),
             reference_whitened_norm_translation=float(np.sqrt(e[3:]@np.linalg.solve(T,e[3:]))),
             reference_distance_mm=float(np.linalg.norm(target[:3,3])*1000),
             belief_distance_mm=float(np.linalg.norm(Z[:3,3])*1000),
             translation_eigenvalues_m2=np.linalg.eigvalsh(T).tolist(),
             mean_log=mu.tolist(), covariance=C.tolist(), reference_error_log=e.tolist(),
             reference_relative_matrix=target.tolist())
    return r


def read(mode, ref):
    root = P/f'full_{mode}_receiver'
    source = P/f'full_{mode}_source'
    for p in [root/'manifest.json', root/'resolved_config.json', root/'receiver/summary.json']:
        pin(p)
    native = source/'kimera_odometry.csv'; pin(native)
    times = {}
    for r in csv.DictReader(native.open()):
        assert r['field.child_frame_id'] == 'cpt7_imu'
        times[int(r['field.header.seq'])] = int(r['field.header.stamp'])
    online = root/'receiver/online_states.csv'; pin(online)
    with online.open() as f: origin = int(next(csv.DictReader(f))['timestamp_ns'])
    events, receipt = root/'events.csv', root/'receipts.jsonl'
    eh, rh = hashlib.sha256(), hashlib.sha256()
    rows = {}
    max_rounding = 0
    with events.open('rb') as f, receipt.open('rb') as g:
        for line, (raw, rr) in enumerate(zip_longest(f,g), 1):
            assert raw is not None and rr is not None
            eh.update(raw); rh.update(rr)
            if not raw.startswith(b'B,'): continue
            a = raw.decode().strip().split(',')
            if a[2] != 'kimera_front' or tuple(map(int,a[6:8])) not in EDGES: continue
            assert len(a) == 52
            meta = json.loads(rr)
            assert meta['event'] == line-1 and meta['kind'] == 'B'
            assert meta['receipt_ros_ns'] == int(a[1])
            m = meta['metadata']; update = int(m['source_update'])
            ordinal, revision, i, j, ti, tj = map(int,a[4:10])
            assert m['source'] == a[2] and int(m['source_revision']) == revision
            for index, stamp in [(i,ti), (j,tj)]:
                max_rounding = max(max_rounding, abs(times[index]-stamp))
                assert abs(times[index]-stamp) <= 1000
            key = i,j,revision
            assert key not in rows
            rows[key] = dict(mode=mode, source=a[2], session=a[3], from_id=i,to_id=j,
                revision=revision,ordinal=ordinal,update_id=update,start_ns=ti,end_ns=tj,
                arrival_ns=int(a[1]),event=line,source_age_s=(times[update]-tj)*1e-9,
                physical_start_s=(ti-START)*1e-9,physical_end_s=(tj-START)*1e-9,
                arrival_s=(int(a[1])-START)*1e-9,
                _mu=np.asarray(a[10:16],float),_C=np.asarray(a[16:],float).reshape(6,6))
    HASHES[str(events)] = eh.hexdigest(); HASHES[str(receipt)] = rh.hexdigest()
    assert len(rows) == 75
    for i,j in EDGES:
        assert sorted(r for a,b,r in rows if (a,b)==(i,j)) == list(range(25))

    # Source log is an independent join: same-snapshot native poses, exported
    # mean and covariance; no mixing states from different backend updates.
    log = source/'estimators.log'; pin(log)
    checks = dict(source_pose_to_mean_max=0., source_mean_to_wire_max=0.,
                  source_covariance_to_wire_max=0., archive_mean_max=0., archive_covariance_max=0.)
    matched = set()
    for tag,a in geo.rows(log, ['KIMERA_CBS_REVISION_EDGE_ROW']):
        update,i,j,rev = int(a[0]),int(a[2]),int(a[3]),int(a[6])
        if (i,j) not in EDGES: continue
        key = i,j,rev; r=rows[key]
        assert key not in matched and update == r['update_id'] and len(a)==64
        matched.add(key)
        A,B = geo.pose(a[8:15]),geo.pose(a[15:22])
        mu=np.asarray(a[22:28],float); C=np.asarray(a[28:],float).reshape(6,6)
        checks['source_pose_to_mean_max']=max(checks['source_pose_to_mean_max'],float(np.max(abs(geo.inv(A)@B-geo.exp(mu)))))
        checks['source_mean_to_wire_max']=max(checks['source_mean_to_wire_max'],float(np.max(abs(mu-r['_mu']))))
        checks['source_covariance_to_wire_max']=max(checks['source_covariance_to_wire_max'],float(np.max(abs(C-r['_C']))))
    assert matched == set(rows)

    # Every selected factor is either excluded or absorbed before capture+8 s.
    # Scan the retained first 8 s of committed receiver transactions only.
    progress=root/'receiver/stream_progress.csv'; pin(progress)
    cutoff=0
    for r in csv.DictReader(progress.open()):
        if int(r['arrival_ns']) > START+8_000_000_000: break
        cutoff=max(cutoff,int(r['event']))
    matched=set(); absorbed=[]; stored_files=[]
    for folder in sorted((root/'receiver/archive/commits').iterdir(),key=lambda p:int(p.name)):
        event=int(folder.name)
        if event>cutoff: break
        path=folder/'records.txt'
        relevant=False
        for a in csv.reader(path.open()):
            if a[0]=='belief' and a[2]=='kimera_front' and tuple(map(int,a[4:6])) in EDGES:
                i,j,rev=map(int,a[4:7]); key=i,j,rev; r=rows[key]
                assert key not in matched and len(a)==55
                matched.add(key); relevant=True
                assert a[3]==r['session'] and int(a[7])==r['ordinal']
                assert list(map(int,a[8:11]))==[r['start_ns'],r['end_ns'],r['arrival_ns']]
                mu=np.asarray(a[13:19],float); C=np.asarray(a[19:],float).reshape(6,6)
                checks['archive_mean_max']=max(checks['archive_mean_max'],float(np.max(abs(mu-r['_mu']))))
                checks['archive_covariance_max']=max(checks['archive_covariance_max'],float(np.max(abs(C-r['_C']))))
                r.update(receiver_action=a[1],commit_event=event,insertion_ns=int(a[11]),slot=int(a[12]))
            if a[0]=='absorbed' and a[1]=='kimera_front' and tuple(map(int,a[3:5])) in EDGES:
                relevant=True
                absorbed.append(dict(event=event,from_id=int(a[3]),to_id=int(a[4]),
                                     revision=int(a[5]),slot=int(a[6]),arrival_ns=int(a[7])))
        if relevant:
            pin(path);stored_files.append(str(path))
    assert matched == set(rows)
    assert len(absorbed)==1 and (absorbed[0]['from_id'],absorbed[0]['to_id'],absorbed[0]['revision'])==(2,3,24)
    for i,j in EDGES:
        counts=Counter(rows[i,j,rev]['receiver_action'] for rev in range(25))
        assert counts == (Counter(before_initialization=25) if i<2 else Counter(insert=1,replace=24)),counts
    for name,value in checks.items(): assert value < 1e-10,(name,value)
    marker=root/'receiver/archive/ids/kimera_front'/rows[2,3,24]['session']/'2_3'
    pin(marker); lines=marker.read_text().splitlines(); a=lines[1].split()
    assert int(lines[0])==absorbed[0]['event'] and int(a[0])==24
    assert len(a)==58
    assert np.max(abs(np.asarray(a[6:22],float).reshape(4,4)-geo.exp(rows[2,3,24]['_mu'])))<1e-10
    assert np.array_equal(np.asarray(a[22:],float).reshape(6,6),rows[2,3,24]['_C'])
    for r in rows.values(): measure(r,ref)
    result = dict(origin_ns=origin,origin_s=(origin-START)*1e-9,versions=len(rows),
        max_endpoint_rounding_ns=max_rounding,payload_checks=checks,absorbed=absorbed,
        committed_actions=dict(Counter(r['receiver_action'] for r in rows.values())),
        scanned_through_event=cutoff,commit_files=stored_files)
    return rows,result


def main():
    out=HERE/'outputs'; out.mkdir(exist_ok=False)
    assert geo.self_test()
    pin(GEO);pin(BAG);pin(Path(__file__));pin(HERE/'PROTOCOL.txt')
    ref=geo.Reference(BAG)
    allrows=[]; audit={}; selected={}; comparison={}
    for mode in ('off','on'):
        rows,audit[mode]=read(mode,ref); allrows.extend(rows.values())
        selected[mode]={f'{i}->{j}':{str(rev):rows[i,j,rev] for rev in (0,1,2,3,5,10,20,24)} for i,j in EDGES}
    for i,j in EDGES:
        off=[r for r in allrows if r['mode']=='off' and (r['from_id'],r['to_id'])==(i,j)]
        on=[r for r in allrows if r['mode']=='on' and (r['from_id'],r['to_id'])==(i,j)]
        for a,b in zip(off,on):
            assert (a['revision'],a['start_ns'],a['end_ns'],a['update_id'])==(b['revision'],b['start_ns'],b['end_ns'],b['update_id'])
        comparison[f'{i}->{j}']={str(a['revision']):dict(
            mean_off_on_translation_mm=float(np.linalg.norm(geo.log(geo.inv(geo.exp(a['mean_log']))@geo.exp(b['mean_log']))[3:])*1000),
            trace_on_over_off=b['translation_trace_m2']/a['translation_trace_m2'],
            error_on_over_off=b['translation_error_mm']/a['translation_error_mm']) for a,b in zip(off,on)}
    dump(out/'audit.json',dict(cases=audit,selected=selected,off_on=comparison,
        max_reference_bracket_ns=ref.max_bracket_ns,input_sha256=HASHES))
    dump(out/'all_versions.json',allrows)
    scalar=[k for k,v in allrows[0].items() if not isinstance(v,(list,dict))]
    with (out/'all_versions.csv').open('x') as f:
        w=csv.DictWriter(f,fieldnames=scalar);w.writeheader()
        w.writerows({k:r[k] for k in scalar} for r in allrows)
    fig,axes=plt.subplots(2,3,figsize=(15,8),sharex=True)
    colors=dict(off='#2673B8',on='#D65D24')
    for col,(i,j) in enumerate(EDGES):
        for mode in ('off','on'):
            r=[r for r in allrows if r['mode']==mode and (r['from_id'],r['to_id'])==(i,j)]
            age=[x['source_age_s'] for x in r]
            for row,key in enumerate(('translation_error_mm','translation_rss_sigma_mm')):
                axes[row,col].plot(age,[x[key] for x in r],'-o',ms=3,color=colors[mode],label='CBS '+mode.upper())
        axes[0,col].set_title(f'Kimera {i} → {j}\n'+('EXCLUDED before receiver initialization' if i<2 else 'INSERTED + 24 REPLACEMENTS'),fontsize=11)
        for ax in axes[:,col]: ax.set_yscale('log');ax.grid(alpha=.25);ax.legend(fontsize=9)
        axes[1,col].set_xlabel('Age of this edge in source smoother (s)')
    axes[0,0].set_ylabel('Relative translation error vs IE-TC (mm)')
    axes[1,0].set_ylabel('√ translation covariance trace (mm)')
    fig.suptitle('SPX-3: the same three startup beliefs through every revision',fontsize=16)
    fig.text(.5,.018,'Shared-CPT7 IE-TC reference; not independent GT. Different y-scales: compare numbers, not slopes.\nUncertainty is the sender’s outgoing payload; these are not per-update receiver residuals.',ha='center',fontsize=10)
    fig.tight_layout(rect=(0,.08,1,.94))
    fig.savefig(out/'startup_revisions.png',dpi=160)
    fig.savefig(out/'startup_revisions.svg')
    plt.close(fig)
    # Confirm saved evidence was not modified by the analysis.
    for path,expected in HASHES.items(): assert geo.sha(Path(path))==expected,path
    print(json.dumps(dict(cases={k:{n:v for n,v in a.items() if n!='commit_files'} for k,a in audit.items()},
                         max_reference_bracket_ns=ref.max_bracket_ns,verified_input_files=len(HASHES)),indent=2))


if __name__=='__main__': main()
