"""Full CP-V4 Leica-2 four-path overlays with saved Velodyne environment."""
import hashlib
import json
from pathlib import Path
import numpy as np
import rerun as rr
import rerun.blueprint as b

H=Path(__file__).resolve().parent
BASE=H.parent
SRC=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_leica2_velodyne_full_cpv4_20261024/prepared/full_on_source')
COLORS={'reference':[75,155,255],'glim_initial':[50,215,100],
        'kimera':[255,175,35],'fusion_online':[255,65,85]}
LABELS={'reference':'Reference_IE_TC','glim_initial':'GLIM_first_published',
        'kimera':'Kimera_ROS_odometry','fusion_online':'Fusion_ONLINE'}


def sha(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        while data:=f.read(8*1024*1024):h.update(data)
    return h.hexdigest()


def main():
    dest=H/'leica2_full_scene.rrd'
    assert not dest.exists()
    inputs={}
    def pin(p):
        p=Path(p);inputs[str(p)]=sha(p);return p
    meta=json.loads(pin(BASE/'data_checks.json').read_text())
    zp=pin(BASE/'trajectories.npz');assert sha(zp)==meta['npz_sha256']
    z=np.load(zp)
    pin(Path(__file__));pin(H/'PROTOCOL.txt')
    for name in ('on_glim_initial','off_glim_initial','on_kimera','off_kimera','on_fusion_online','off_fusion_online'):
        p=pin(meta['products'][name]['path'])
        assert sha(p)==meta['products'][name]['sha256']
    # Saved clouds are submap-local float32 XYZ. T_world_origin places them in
    # native GLIM world; the fixed CP-V10 fit then places them in reference ENU.
    alignment=meta['products']['on_glim_initial']
    rotation=np.asarray(alignment['alignment_rotation'])
    translation=np.asarray(alignment['alignment_translation_m'])
    anchor=np.asarray(meta['origin_reference_enu_m'])
    odom=np.loadtxt(pin(SRC/'odom_imu.txt'))
    batches=[];errors=[];submaps=[]
    for path in sorted((SRC/'glim_dump').glob('[0-9]*/points_compact.bin')):
        pin(path);textpath=pin(path.with_name('data.txt'))
        lines=textpath.read_text().splitlines()
        def matrix(label):
            index=lines.index(label+': ')
            m=np.asarray([[float(v) for v in line.split()] for line in lines[index+1:index+5]])
            assert np.allclose(m[3],[0,0,0,1])
            assert np.max(np.abs(m[:3,:3].T@m[:3,:3]-np.eye(3)))<3e-6
            return m
        world=matrix('T_world_origin')
        stamps=np.asarray([float(s.split()[1]) for s in lines if s.startswith('stamp: ')])
        assert len(stamps)>0
        idx=int(np.argmin(np.abs(odom[:,0]-stamps[0])))
        assert abs(odom[idx,0]-stamps[0])<1e-4
        imu=matrix('T_world_lidar')@matrix('T_lidar_imu')
        error=float(np.linalg.norm(imu[:3,3]-odom[idx,1:4]));errors.append(error)
        assert error<1e-3,(path,error)
        raw=np.fromfile(path,dtype='<f4').reshape(-1,3).astype(np.float64)
        assert np.isfinite(raw).all()
        points=(raw@world[:3,:3].T+world[:3,3])@rotation.T+translation-anchor
        assert np.isfinite(points).all()
        batches.append(points.astype(np.float32))
        submaps.append({'id':path.parent.name,'points':len(points),'frame_check_m':error,
                        'first_s':float(stamps[0]-meta['epoch_ns']*1e-9),
                        'last_s':float(stamps[-1]-meta['epoch_ns']*1e-9)})
    cloud=np.concatenate(batches)
    # Exact saved environment; no point thinning, alternate map or new scan reconstruction.
    assert np.array_equal(np.load(H/'context_cloud.npz')['position'],cloud)
    allpos=np.concatenate([z[k] for k in z.files if k.endswith('_p')])
    lower,upper=allpos.min(0),allpos.max(0)
    center=(lower+upper)/2;extent=float(np.max(upper-lower))
    eye=b.EyeControls3D(kind=b.Eye3DKind.Orbital,
        position=center+np.asarray([0,-.5*extent,1.25*extent]),look_target=center,eye_up=[0,0,1])
    views=[]
    for mode in ('off','on'):
        views.append(b.Spatial3DView(origin='/scene',
            contents=['+ /scene/environment/**',f'+ /scene/{mode}/**'],
            name='CBS '+mode.upper()+' | FOUR FULL TRAJECTORIES + VELODYNE',eye_controls=eye,
            line_grid=b.LineGrid3D(visible=True,spacing=1,stroke_width=.5,color=[65,75,78]),
            background=[16,20,23]))
    blueprint=b.Blueprint(b.Vertical(
        b.Tabs(*views,b.TextDocumentView(origin='/notes',name='Reading the overlay'),active_tab=0),
        b.TextDocumentView(origin='/legend',name='Full paths | same coordinate frame | no smoothing'),row_shares=[.92,.08]),
        b.BlueprintPanel(state='collapsed'),b.SelectionPanel(state='collapsed'),
        b.TimePanel(state='hidden'),auto_layout=False,auto_views=False)
    rr.init('LEICA-2 FULL SCENE | Four trajectories + Velodyne',
            recording_id='cpv11-full-scene-final-20261026',strict=True)
    rr.save(dest,default_blueprint=blueprint)
    rr.log('/scene',rr.ViewCoordinates.RIGHT_HAND_Z_UP,static=True)
    rr.log('/scene/environment/Velodyne_saved_ON_map',rr.Points3D(cloud,
           colors=[125,140,150,100],radii=rr.Radius.ui_points(.35)),static=True)
    counts={}
    for mode in ('off','on'):
        for role in COLORS:
            key='reference' if role=='reference' else mode+'_'+role
            p=z[key+'_p'].astype(np.float32);t=z[key+'_t_ns']
            gap=50_000_000 if role=='reference' else 500_000_000
            pieces=np.split(np.arange(len(p)),np.flatnonzero(np.diff(t)>gap)+1)
            lines=[p[ix] for ix in pieces if len(ix)>1]
            ent=f'/scene/{mode}/{LABELS[role]}'
            rr.log(ent,rr.LineStrips3D(lines,colors=COLORS[role],radii=rr.Radius.ui_points(2.2)),static=True)
            # Retain isolated points and every original vertex for fine-scale inspection.
            rr.log(ent+'/samples',rr.Points3D(p,colors=COLORS[role],radii=rr.Radius.ui_points(.65)),static=True)
            counts[mode+'_'+role]={'vertices':len(p),'segments':sum(len(ix)-1 for ix in pieces)}
    rr.log('/legend',rr.TextDocument('BLUE: IE-TC reference | GREEN: GLIM first-published | ORANGE: Kimera | RED: fusion ONLINE\nAll FOUR FULL paths overlaid. Drag to orbit; scroll to zoom; OFF / ON tabs. Grey: saved Velodyne map (GLIM context, not ground truth).'),static=True)
    rr.log('/notes',rr.TextDocument('''# Four COMPLETE trajectories in one scene

Choose CBS OFF or CBS ON. Each tab overlays reference, first-published GLIM,
Kimera ROS odometry and fusion ONLINE over the SAME saved Velodyne environment.
The main view is the complete path, not a selected time window or growing trail.
Orbit and zoom to see the separation and fine jagged motion; all native points
are retained. Use the blueprint/entity tree eye toggles to hide a path or map.

All paths share reference ENU orientation, common prism point and common origin.
Each estimate uses ONE fixed no-scale SE(3) fit from the CP-V10 common mask.
No per-segment alignment, start-position normalization, smoothing or resampling.
Visible path separation is residual error; the paths are not forced to coincide.
Some source initialization/tail and Kimera publication gaps remain visible.

The grey cloud comprises ALL saved CBS-ON GLIM Velodyne submaps, reconstructed
from saved submap transforms and placed using the SAME rigid transform as ON
GLIM. The identical cloud is used for OFF to avoid changing the visual backdrop.
This map can contain GLIM reconstruction errors and is NOT independent GT.
Its finalized submap poses provide environment context only; neither GLIM's
first-published path nor fusion ONLINE has been replaced with refined output.
IE-TC shares CPT7; no independent prism-ground-truth claim. ONLINE connection
jitter is not proof of physical flight jitter or the latest revised graph shape.
No estimator replay, new mapping run, covariance change or source rerun.
''',media_type=rr.MediaType.MARKDOWN),static=True)
    assert all(sha(Path(path))==digest for path,digest in inputs.items())
    summary={'submaps':submaps,'cloud_points':len(cloud),'cloud_thinning':False,
             'max_lidar_imu_frame_error_m':max(errors),'trajectory_counts':counts,
             'input_sha256':inputs,'inputs_unchanged':True,'trajectory_resampling':False,
             'cloud_alignment':alignment,'common_origin':anchor.tolist(),'recording':str(dest)}
    with (H/'scene_build_checks.json').open('x') as f:json.dump(summary,f,indent=2)
    print(json.dumps({k:summary[k] for k in ('cloud_points','max_lidar_imu_frame_error_m','inputs_unchanged','recording')},indent=2))


if __name__=='__main__':main()
