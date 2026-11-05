"""Full untouched vertex overlays: fixed original alignment for both outputs."""
import hashlib,json
from pathlib import Path
import numpy as np
import rerun as rr
import rerun.blueprint as b
H=Path(__file__).resolve().parent
BASE=H.parent/'leica2_velodyne_rerun_cpv11_20261026'
COLORS={'reference':[75,155,255],'absolute':[255,65,85],'chained':[193,112,255],
        'glim_initial':[50,215,100],'kimera':[255,175,35]}
LABELS={'reference':'Reference_IE_TC','absolute':'Fusion_absolute_GRAPH_ONLINE',
        'chained':'Fusion_CHAINED_incremental','glim_initial':'GLIM_first_published','kimera':'Kimera_ROS_incremental'}
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def load():
    z=np.load(BASE/'trajectories.npz');new=np.load(H/'comparison_arrays.npz')
    epoch=int(json.loads((BASE/'data_checks.json').read_text())['epoch_ns'])
    products={}
    for mode in ('off','on'):
        for role in COLORS:
            if role in ('absolute','chained'):p,t=new[mode+'_'+role+'_display_p'],new[mode+'_timestamp_ns']
            else:
                key='reference' if role=='reference' else mode+'_'+role
                p,t=z[key+'_p'],z[key+'_t_ns']+epoch
            products[(mode,role)]=(p.astype(np.float32),t)
    cloud=np.load(BASE/'full_overlay/context_cloud.npz')['position']
    return products,cloud
def strips(p,t,role):
    gap=50_000_000 if role=='reference' else 500_000_000
    pieces=np.split(np.arange(len(p)),np.flatnonzero(np.diff(t)>gap)+1)
    return [p[ix] for ix in pieces if len(ix)>1]
def main():
    dest=H/'leica2_incremental_comparison.rrd';assert not dest.exists()
    paths=[Path(__file__),H/'PROTOCOL.txt',H/'results.json',H/'reconstruction_checks.json',
        H/'comparison_arrays.npz',BASE/'trajectories.npz',BASE/'data_checks.json',
        BASE/'full_overlay/context_cloud.npz',BASE/'full_overlay/scene_build_checks.json']
    pins={str(p):sha(p) for p in paths}
    meta=json.loads((BASE/'data_checks.json').read_text())
    assert sha(BASE/'trajectories.npz')==meta['npz_sha256']
    products,cloud=load();assert len(cloud)==776491
    old=np.load(BASE/'trajectories.npz')
    for mode in ('off','on'):
        assert np.allclose(products[mode,'absolute'][0],old[mode+'_fusion_online_p'],atol=1e-4,rtol=0)
        assert np.array_equal(products[mode,'absolute'][1],old[mode+'_fusion_online_t_ns']+int(meta['epoch_ns']))
    allpos=np.concatenate([p for p,t in products.values()]);lo,hi=allpos.min(0),allpos.max(0)
    center=(lo+hi)/2;extent=float(np.max(hi-lo))
    eye=b.EyeControls3D(kind=b.Eye3DKind.Orbital,position=center+np.array([0,-.55*extent,1.25*extent]),look_target=center,eye_up=[0,0,1])
    views=[]
    for context in (False,True):
        for mode in ('off','on'):
            content=['+ /scene/environment/**',f'+ /scene/{mode}/**']
            if not context:content += [f'- /scene/{mode}/{LABELS[k]}/**' for k in ('glim_initial','kimera')]
            views.append(b.Spatial3DView(origin='/scene',contents=content,eye_controls=eye,
                name='CBS '+mode.upper()+(' | + native estimators' if context else ' | ABSOLUTE vs INCREMENTAL'),
                line_grid=b.LineGrid3D(visible=True,spacing=1,stroke_width=.5,color=[65,75,78]),background=[16,20,23]))
    blueprint=b.Blueprint(b.Vertical(b.Tabs(*views,b.TextDocumentView(origin='/notes',name='Read me'),active_tab=0),
        b.TextDocumentView(origin='/legend',name='Full trajectories | identical alignment | no smoothing'),row_shares=[.9,.1]),
        b.BlueprintPanel(state='collapsed'),b.SelectionPanel(state='collapsed'),b.TimePanel(state='hidden'),auto_layout=False,auto_views=False)
    rr.init('LEICA-2 | Absolute vs chained incremental | CP-V23',recording_id='cpv23-leica2-incremental-full-20261027',strict=True)
    rr.save(dest,default_blueprint=blueprint)
    rr.log('/scene',rr.ViewCoordinates.RIGHT_HAND_Z_UP,static=True)
    rr.log('/scene/environment/Velodyne_saved_ON_map',rr.Points3D(cloud,colors=[125,140,150,100],radii=rr.Radius.ui_points(.35)),static=True)
    counts={}
    for (mode,role),(p,t) in products.items():
        ent=f'/scene/{mode}/{LABELS[role]}';lines=strips(p,t,role)
        rr.log(ent,rr.LineStrips3D(lines,colors=COLORS[role],radii=rr.Radius.ui_points(2.2)),static=True)
        rr.log(ent+'/samples',rr.Points3D(p,colors=COLORS[role],radii=rr.Radius.ui_points(.65)),static=True)
        counts[mode+'_'+role]={'vertices':len(p),'edges':sum(len(line)-1 for line in lines),'pieces':len(lines)}
    rr.log('/legend',rr.TextDocument('BLUE: IE-TC | RED: absolute graph ONLINE | PURPLE: chained incremental\nOFF / ON tabs. Extra tabs include GREEN: GLIM first-published and ORANGE: Kimera. Same original alignment for BOTH fusion paths; no smoothing. Grey map is saved GLIM Velodyne context, not GT.'),static=True)
    rr.log('/notes',rr.TextDocument('''# Full saved-data incremental-publication test — CP-V23

The first two tabs compare COMPLETE original absolute fusion, causal chained
fusion, and IE-TC reference. The next two also show unchanged native GLIM and
Kimera. Orbit / zoom; use the entity tree to toggle paths. No vertex thinning.

Purple is O[k] = O[k-1] inverse(X[k-1|k]) X[k|k]. Both edge endpoints come
from the SAME graph solution available at publication k, not future data.
It omits corrections to the earlier odometry history. RED remains the
original absolute newest graph pose. The receiver and estimator settings
are unchanged; full replayed ONLINE and finalized outputs match exactly.
The separate graph-to-odometry correction reconstructs RED from PURPLE.

Both fusion products use the ORIGINAL absolute fusion rigid no-scale fit
and same prism point and ENU origin. Purple is NOT independently fitted in
this display. Rigid fits used for ATE in the report are separately labeled.
A smoother or shorter purple path does not by itself imply better accuracy.
No extra output delay, low-pass filter, source rerun, Qc tuning, or changed
graph covariance. No new odometry covariance / velocity is assigned.

Native Kimera already uses incremental publication; native GLIM here is
first-published. Original per-estimator fixed fits remain unchanged.
All 776491 original saved ON-map points are shared between both modes.
The map is finalized GLIM reconstruction context, not independent GT.
IE-TC shares CPT7 and is not independent prism GT. Startup/tail omissions
remain; complete saved paths are shown, not only common-mask scoring points.
DELAYED fixed-lag-finalized fusion is scored separately, not drawn as ONLINE.
Offline replay reconstructs causal publication history; this was not a live
co-running sensor experiment. No production output policy was changed.
''',media_type=rr.MediaType.MARKDOWN),static=True)
    assert all(sha(p)==d for p,d in pins.items())
    with (H/'scene_build_checks.json').open('x') as f:json.dump({'pass_':True,'input_sha256':pins,'counts':counts,'cloud_points':len(cloud),'original_absolute_alignment_check':True,'recording':str(dest)},f,indent=2)
    print(dest)
if __name__=='__main__':main()
