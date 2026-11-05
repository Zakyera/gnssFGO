"""One portable RRD per pair, containing full OFF/ON method2 overlays."""
import hashlib
import json
import sys
from pathlib import Path
import numpy as np
import rerun as rr
import rerun.blueprint as b

H = Path(__file__).resolve().parent
OUT = Path('/home/yeranis/results/rerun_method2_all_sequences_20261029')
COLORS = dict(reference=[75,155,255], glim=[50,215,100],
              kimera=[255,175,35], fusion=[255,65,85])
LABELS = dict(reference='Reference_IE_TC', glim='GLIM_method2',
              kimera='Kimera_native_incremental', fusion='Fusion_method2')


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        while data := f.read(8*1024*1024): h.update(data)
    return h.hexdigest()


def pieces(p, t, role):
    gap = 50_000_000 if role == 'reference' else 500_000_000
    ix = np.split(np.arange(len(p)), np.flatnonzero(np.diff(t)>gap)+1)
    return [p[i] for i in ix if len(i)>1]


def main(case):
    meta = json.loads((H/(case+'.json')).read_text())
    assert sha(H/(case+'.npz')) == meta['trajectory_npz_sha256']
    assert sha(H/(case+'_cloud.npz')) == meta['cloud_npz_sha256']
    z = np.load(H/(case+'.npz'))
    cloud = np.load(H/(case+'_cloud.npz'))['position']
    OUT.mkdir(parents=True, exist_ok=True)
    dest = OUT/(case+'_method2.rrd')
    assert not dest.exists()
    pp = np.concatenate([z[k] for k in z.files if k.endswith('_p')])
    lo,hi = pp.min(0),pp.max(0)
    center = (lo+hi)/2
    extent = max(float(np.max(hi-lo)), 1.0)
    eye = b.EyeControls3D(kind=b.Eye3DKind.Orbital,
        position=center+np.array([0,-.5*extent,1.25*extent]),
        look_target=center, eye_up=[0,0,1])
    views = [b.Spatial3DView(origin='/scene',
        contents=['+ /scene/environment/**', f'+ /scene/{mode}/**'],
        name=f'CBS {mode.upper()} | METHOD 2 | {meta["sequence"]} {meta["sensor"]}',
        eye_controls=eye,
        line_grid=b.LineGrid3D(visible=True,spacing=1,stroke_width=.5,color=[65,75,78]),
        background=[16,20,23]) for mode in ('off','on')]
    bp = b.Blueprint(b.Vertical(
        b.Tabs(*views,b.TextDocumentView(origin='/notes',name='Read me / results'),active_tab=0),
        b.TextDocumentView(origin='/legend',name='Four FULL paths | METHOD 2 | no smoothing'),
        row_shares=[.92,.08]), b.BlueprintPanel(state='collapsed'),
        b.SelectionPanel(state='collapsed'), b.TimePanel(state='hidden'),
        auto_layout=False, auto_views=False)
    rec = rr.RecordingStream(f'{meta["sequence"]} {meta["sensor"]} | METHOD 2 | OFF + ON',
        recording_id='cpv30-'+case+'-20261029')
    rec.save(dest, default_blueprint=bp)
    rec.log('/scene',rr.ViewCoordinates.RIGHT_HAND_Z_UP,static=True)
    rec.log('/scene/environment/Saved_ON_GLiM_map_context',
        rr.Points3D(cloud,colors=[125,140,150,100],radii=rr.Radius.ui_points(.35)),static=True)
    counts = {}
    for mode in ('off','on'):
        for role in COLORS:
            key = 'reference' if role=='reference' else mode+'_'+role
            p,t = z[key+'_p'].astype(np.float32),z[key+'_t_ns']
            strips = pieces(p,t,role)
            entity = f'/scene/{mode}/{LABELS[role]}'
            rec.log(entity,rr.LineStrips3D(strips,colors=COLORS[role],
                radii=rr.Radius.ui_points(2.2)),static=True)
            rec.log(entity+'/samples',rr.Points3D(p,colors=COLORS[role],
                radii=rr.Radius.ui_points(.65)),static=True)
            counts[mode+'_'+role] = dict(vertices=len(p),strips=len(strips),
                edges=sum(len(line)-1 for line in strips))
    rec.log('/legend',rr.TextDocument(
        'BLUE: IE-TC reference | GREEN: GLIM method 2 | ORANGE: Kimera incremental | RED: fusion method 2\n'
        'All FULL paths. OFF / ON tabs; drag to orbit, scroll to zoom. Grey: saved GLIM map context, not ground truth. Original WNOA Qc x1.0.'),static=True)
    rows = []
    for role,label in [('glim','GLIM'),('kimera','Kimera'),('fusion','Fusion')]:
        off,on = (meta['products'][mode+'_'+role] for mode in ('off','on'))
        rows.append(f'| {label} | {off["common_ATE_m"]:.4f} | {on["common_ATE_m"]:.4f} | '
                    f'{off["path_with_gap_chords_m"]:.2f} / {on["path_with_gap_chords_m"]:.2f} |')
    notes = f'''# {meta['sequence']} / {meta['sensor']} — METHOD 2

Choose CBS OFF or CBS ON. Each tab overlays FOUR complete saved trajectories
in one 3D scene. Full paths remain visible without playback. Open the Blueprint
panel and use eye toggles to hide any trajectory or the grey environment.

Green GLIM and red fusion accumulate SAME-update relative transformations.
Orange Kimera is its native incremental ROS odometry, not integrated twice.
These are exactly the CP-V28 method2 products used for both RMSE and path.
They are NOT first-published absolute paths or fixed-lag-finalized paths.

| Estimator | Common RMSE OFF (m) | Common RMSE ON (m) | Full native path OFF / ON (m) |
|---|---:|---:|---:|
{chr(10).join(rows)}

Path values above include long-gap chords as in CP-V28's full-saved table.
The drawing does NOT connect estimator gaps >500ms or reference gaps >50ms;
all isolated points remain. Path length is estimated, not physical travel.
Common-mask coverage: {meta['common_coverage']['supported_s']:.1f}s of
{meta['duration_s']:.2f}s capture. The display is NOT cropped to this mask.

All paths use the common prism point and reference ENU origin. Each estimate
has ONE rigid SE3 alignment (no scaling) from the frozen CP-V28 common mask.
No local realignment, time offset tuning, smoothing, or trajectory thinning.
This offline alignment aids evaluation; it is not an onboard correction.

Grey cloud: {meta['cloud_points']:,} points from {len(meta['submaps'])} available
saved CBS-ON {meta['sensor']} submaps, identical backdrop in both tabs.
It remains in corrected GLIM map coordinates, placed into reference ENU with
the original CP-V10 ON absolute-GLIM fit, NOT the drifting method2 fit.
No cloud warping. Saved map coverage can be partial; it is context, NOT
independent truth and NOT a map reconstructed along these incremental paths.

IE-TC shares CPT7 IMU, not independent prism ground truth. Original receiver
Qc multiplier 1.0, not 0.25 or 0.1. Same output convention does not establish
equal cadence or latency. No source or receiver rerun was needed for this RRD.
The fresh CP-V29 ETH1 OFF experiment and failed ARC prefixes are not mixed in.
'''
    rec.log('/notes',rr.TextDocument(notes,media_type=rr.MediaType.MARKDOWN),static=True)
    rec.flush()
    with (H/(case+'_build.json')).open('x') as f:
        json.dump(dict(case=case,recording=str(dest),trajectory_counts=counts,
            cloud_points=len(cloud),rrd_bytes=dest.stat().st_size,
            build_script_sha256=sha(Path(__file__))),f,indent=2)
    print(json.dumps(dict(case=case,recording=str(dest),bytes=dest.stat().st_size)),flush=True)


if __name__=='__main__': main(sys.argv[1])
