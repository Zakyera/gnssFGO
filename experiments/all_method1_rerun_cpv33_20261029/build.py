"""One portable RRD per pair, containing full OFF/ON method1 overlays."""
import hashlib
import json
import sys
from pathlib import Path
import numpy as np
import rerun as rr
import rerun.blueprint as b

H = Path(__file__).resolve().parent
OUT = Path('/home/yeranis/results/rerun_method1_all_sequences_20261029')
COLORS = dict(reference=[75,155,255], glim=[50,215,100],
              kimera=[255,175,35], fusion=[255,65,85])
LABELS = dict(reference='Reference_IE_TC', glim='GLIM_absolute_first_published',
              kimera='Kimera_absolute_backend', fusion='Fusion_ONLINE_absolute')


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
    assert sha(Path(meta['cloud_path'])) == meta['cloud_npz_sha256']
    z = np.load(H/(case+'.npz'))
    cloud = np.load(Path(meta['cloud_path']))['position']
    OUT.mkdir(parents=True, exist_ok=True)
    dest = OUT/(case+'_method1.rrd')
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
        name=f'CBS {mode.upper()} | METHOD 1 | {meta["sequence"]} {meta["sensor"]}',
        eye_controls=eye,
        line_grid=b.LineGrid3D(visible=True,spacing=1,stroke_width=.5,color=[65,75,78]),
        background=[16,20,23]) for mode in ('off','on')]
    bp = b.Blueprint(b.Vertical(
        b.Tabs(*views,b.TextDocumentView(origin='/notes',name='Read me / results'),active_tab=0),
        b.TextDocumentView(origin='/legend',name='Four FULL paths | METHOD 1 | no smoothing'),
        row_shares=[.92,.08]), b.BlueprintPanel(state='collapsed'),
        b.SelectionPanel(state='collapsed'), b.TimePanel(state='hidden'),
        auto_layout=False, auto_views=False)
    rec = rr.RecordingStream(f'{meta["sequence"]} {meta["sensor"]} | METHOD 1 | OFF + ON',
        recording_id='cpv33-'+case+'-20261029')
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
        'BLUE: IE-TC reference | GREEN: GLIM first absolute | ORANGE: Kimera absolute backend | RED: fusion ONLINE absolute\n'
        'All FULL paths. OFF / ON tabs; drag to orbit, scroll to zoom. Grey: saved GLIM map context, not ground truth. Original WNOA Qc x1.0.'),static=True)
    rows = []
    for role,label in [('glim','GLIM'),('kimera','Kimera absolute backend'),('fusion','Fusion ONLINE')]:
        off,on = (meta['products'][mode+'_'+role] for mode in ('off','on'))
        rows.append(f'| {label} | {off["common_ATE_m"]:.4f} / {on["common_ATE_m"]:.4f} | '
                    f'{off["common_RPE_translation_m"]:.4f} / {on["common_RPE_translation_m"]:.4f} | '
                    f'{off["common_RPE_rotation_deg"]:.3f} / {on["common_RPE_rotation_deg"]:.3f} | '
                    f'{off["common_path_m"]:.2f} / {on["common_path_m"]:.2f} |')
    native_rows = []
    for role in ('glim','kimera','fusion'):
        off,on = (meta['products'][mode+'_'+role] for mode in ('off','on'))
        native_rows.append(f'| {role} | {off["path_with_gap_chords_m"]:.2f} / {on["path_with_gap_chords_m"]:.2f} | '
                           f'{off["displayed_gap_aware_path_m"]:.2f} / {on["displayed_gap_aware_path_m"]:.2f} | '
                           f'{off["rows"]} / {on["rows"]} |')
    notes = f'''# {meta['sequence']} / {meta['sensor']} — METHOD 1

Choose CBS OFF or CBS ON. Each tab overlays FOUR complete saved trajectories.
Full paths remain visible without playback. Open the Blueprint panel and use
eye toggles to hide a trajectory or the grey environment.

Green: GLIM first-published absolute MAIN poses.
Orange: Kimera recovered newest absolute backend poses, NOT its native
incremental ROS output. Red: fusion ONLINE newest-support absolute poses.
Each historical point stays at its original output-time value: later revisions
do NOT replace it. These are CP-V27 Method 1, NOT accumulated increments and
NOT fixed-lag-finalized trajectories. Kimera rounded original logging gives a
conservative prism-position reconstruction bound <=0.504mm.

## Method 1 EVO table — OFF / ON

| Estimator | ATE RMSE (m) | RPE translation (m) | RPE rotation (deg) | Common path (m) |
|---|---:|---:|---:|---:|
{chr(10).join(rows)}

Metrics above come directly from CP-V32 / evo1.36.3. ATE uses one global SE3
alignment, no scale correction. RPE uses consecutive reference-distance pairs
at delta0.5m (a sampled threshold, not exact spatial interpolation).
Common-mask coverage: {meta['common_coverage']['supported_s']:.1f}s of
{meta['duration_s']:.2f}s capture. Common path excludes unsupported gaps.
The FULL display is NOT cropped to the table's common mask.

## FULL native paths — OFF / ON

| Estimator | Connected polyline (m) | Drawn gap-aware polyline (m) | Saved poses |
|---|---:|---:|---:|
{chr(10).join(native_rows)}

Full native lengths are computed with evo trajectory.path_length, not ATE.
The drawing does NOT connect estimator gaps >500ms or reference gaps >50ms;
all isolated points remain. Estimated polyline length is not physical travel:
connecting absolute publication jumps can greatly inflate it.

All paths use the common prism point and reference ENU origin. Each estimate
uses its exact CP-V32 evo rigid SE3 transform from the frozen common mask.
No local realignment, time-offset tuning, smoothing, integration or thinning.
This offline alignment aids comparison; it is not an onboard correction.

Grey cloud: {meta['cloud_points']:,} points from {len(meta['submaps'])} available
saved CBS-ON {meta['sensor']} submaps, identical backdrop in both tabs and the
existing Method 2 recording. The corrected GLIM map is placed into reference
ENU by the original CP-V10 ON absolute-GLIM fit. No cloud warping.
Saved map coverage can be partial; context only, NOT independent ground truth.

IE-TC shares CPT7 IMU, not independent prism ground truth. Original receiver
WNOA Qc multiplier1.0, not0.25 or0.1. Same output convention does not establish
equal cadence, latency or evidence age. No source or receiver rerun.
Fresh CP-V29 ETH1 OFF and failed ARC prefixes are excluded.
'''
    rec.log('/notes',rr.TextDocument(notes,media_type=rr.MediaType.MARKDOWN),static=True)
    rec.flush()
    with (H/(case+'_build.json')).open('x') as f:
        json.dump(dict(case=case,recording=str(dest),trajectory_counts=counts,
            cloud_points=len(cloud),rrd_bytes=dest.stat().st_size,
            build_script_sha256=sha(Path(__file__))),f,indent=2)
    print(json.dumps(dict(case=case,recording=str(dest),bytes=dest.stat().st_size)),flush=True)


if __name__=='__main__': main(sys.argv[1])

