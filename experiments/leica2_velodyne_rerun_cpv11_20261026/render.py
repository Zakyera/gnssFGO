"""Rerun 0.31.4 viewer: full native-rate paths, selected close-up and scrubbing."""
import hashlib
import json
from pathlib import Path
import numpy as np
import rerun as rr
import rerun.blueprint as b

H=Path(__file__).resolve().parent
COLORS={'reference':[75,155,255], 'glim_initial':[50,205,100],
        'kimera':[255,170,45], 'fusion_online':[250,70,85]}
NAMES={'reference':'IE-TC reference', 'glim_initial':'GLIM first-published',
       'kimera':'Kimera ROS odometry', 'fusion_online':'Fusion ONLINE'}
ROLES=tuple(COLORS)


def strips(t,p,gap):
    return [p[i] for i in np.split(np.arange(len(t)),np.flatnonzero(np.diff(t)>gap)+1) if len(i)>1]


def main():
    dest=H/'leica2_velodyne_first_published.rrd'
    assert not dest.exists(), 'Refuse recording overwrite'
    meta=json.loads((H/'data_checks.json').read_text())
    assert hashlib.sha256((H/'trajectories.npz').read_bytes()).hexdigest()==meta['npz_sha256']
    z=np.load(H/'trajectories.npz')
    start,end=(meta['selected_inspection_window'][k] for k in ('start_s','end_s'))
    def curve(mode,role):
        key='reference' if role=='reference' else mode+'_'+role
        return z[key+'_t_ns'],z[key+'_p']
    joined=np.concatenate([curve(m,r)[1] for m in ('off','on') for r in ROLES])
    center=(joined.min(0)+joined.max(0))/2
    extent=float(np.max(np.ptp(joined,axis=0)))
    eye=b.EyeControls3D(kind=b.Eye3DKind.Orbital,position=center+[0,-extent*.8,extent*1.1],
                       look_target=center,eye_up=[0,0,1])
    overview=b.Horizontal(*[b.Spatial3DView(origin='/overview/'+m,contents='$origin/**',
          name='CBS '+m.upper()+' | full native paths',eye_controls=eye) for m in ('off','on')],
          name='Full paths (3D)')
    close_views=[]
    for axes,indices in [('XY',(0,1)),('XZ',(0,2))]:
        pts=np.concatenate([p[(t>=start*1e9)&(t<=end*1e9)][:,indices]
            for m in ('off','on') for r in ROLES for t,p in [curve(m,r)]])
        pts=pts*[1,-1]
        lo,hi=pts.min(0),pts.max(0)
        pad=np.maximum((hi-lo)*.12,.15)
        bounds=b.VisualBounds2D(x_range=[lo[0]-pad[0],hi[0]+pad[0]],
                                y_range=[lo[1]-pad[1],hi[1]+pad[1]])
        close_views.append(b.Horizontal(*[b.Spatial2DView(origin=f'/selected/{axes}/{m}',
            contents='$origin/**',name=f'{m.upper()} | {axes} metres | {start}-{end}s',
            visual_bounds=bounds) for m in ('off','on')]))
    close=b.Vertical(*close_views,name=f'Selected {start}-{end}s close-up')
    window=b.VisibleTimeRange('seconds',start=b.TimeRangeBoundary.cursor_relative(seconds=-10),
                             end=b.TimeRangeBoundary.cursor_relative(seconds=0))
    detail=b.Vertical(
        b.Horizontal(*[b.Spatial3DView(origin='/detail/'+m,contents='$origin/**',
           name=m.upper()+' | previous 10s | native points',time_ranges=[window]) for m in ('off','on')]),
        b.Horizontal(*[b.TimeSeriesView(origin='/steps/'+m,contents='$origin/**',
           name=m.upper()+' | displacement per 100ms (m)',axis_y=b.ScalarAxis(range=[0,.25]))
           for m in ('off','on')]),row_shares=[.7,.3],name='Scrub 10s + step lengths')
    notes=b.TextDocumentView(origin='/notes',name='Legend and interpretation')
    blueprint=b.Blueprint(b.Vertical(b.Tabs(overview,close,detail,notes,active_tab=1),
         b.TextDocumentView(origin='/legend',name='Colour key | native samples, no smoothing'),row_shares=[.93,.07]),
         b.BlueprintPanel(state='collapsed'),b.SelectionPanel(state='collapsed'),
         b.TimePanel(state='collapsed',timeline='seconds',play_state='paused',loop_mode='off'),
         auto_layout=False,auto_views=False)
    rr.init('Leica-2 Velodyne | CP-V4 OFF vs ON | first-published',
            recording_id='cpv11-leica2-velodyne-20261026',strict=True)
    rr.save(dest,default_blueprint=blueprint)
    rr.log('/legend',rr.TextDocument('BLUE: IE-TC reference  |  GREEN: GLIM first-published  |  ORANGE: Kimera ROS odometry  |  RED: fusion ONLINE\nSelected close-up: largest OFF 10s path excess, not a typical window. Scrub uses physical sensor time, not arrival latency.'),static=True)
    rr.log('/notes',rr.TextDocument(f'''# Leica-2 Velodyne: original CP-V4 CBS OFF vs ON

**Blue: IE-TC reference. Green: GLIM first-published. Orange: Kimera ROS odometry. Red: fusion ONLINE.**

All native trajectory vertices retained; no smoothing or estimator resampling.
Each estimate has ONE no-scale rigid SE(3) fit from the CP-V10 common mask.
All represent the same prism point. Coordinates in metres, shifted near zero.
Full plots include startup/tail samples outside the common evaluation mask.
Lines break at estimator gaps >0.5s or reference gaps >0.05s; points remain.

## How to inspect
The opening close-up is **{start}-{end}s**, selected for the largest OFF
10s fusion-minus-reference path excess among fully supported windows.
It is deliberately NOT a typical-window claim. XY is top-down; XZ shows height.
The same bounds are used for OFF and ON. Each dot is a saved sample.
Open Full paths (3D) for the whole recording; drag/orbit and scroll to zoom.
Open Scrub 10s + step lengths, then drag the seconds timeline for any other
interval. Its 3D views show the previous 10 seconds, without resampling.
Step charts are actual consecutive 100ms fusion displacement and reference
displacement at the SAME endpoints, not speed or a filtered path.

## What this can and cannot establish
Red zigzags/jumps belong to the CONNECTED FIRST-PUBLISHED output path.
They do not establish that later corrected active-window states or the drone's
physical flight zigzagged. ONLINE does not replace older published rows.
No finalized fusion or refined GLIM is substituted here. Kimera's ROS output
chains optimized relative motions; equal publication stage is not equal latency.
IE-TC shares the CPT7 IMU: it is not independent prism ground truth.
The existing mean-only covariance alignment approximation is unchanged.
This is saved-data visualization only: no source run or optimizer replay.
''',media_type=rr.MediaType.MARKDOWN),static=True)
    counts={}
    for mode in ('off','on'):
        rr.log('/overview/'+mode,rr.ViewCoordinates.RIGHT_HAND_Z_UP,static=True)
        rr.log('/detail/'+mode,rr.ViewCoordinates.RIGHT_HAND_Z_UP,static=True)
        for role in ROLES:
            t,p=curve(mode,role)
            gap=50_000_000 if role=='reference' else 500_000_000
            color=COLORS[role]
            label=NAMES[role]
            rr.log(f'/overview/{mode}/{label}',rr.LineStrips3D(strips(t,p,gap),
                   colors=color,radii=rr.Radius.ui_points(1.5)),static=True)
            rr.log(f'/overview/{mode}/{label}/samples',rr.Points3D(p,
                   colors=color,radii=rr.Radius.ui_points(.6)),static=True)
            # One native segment per timestamp; visible-time-range displays the trail.
            ok=np.diff(t)<=gap
            entity=f'/detail/{mode}/{label}'
            rr.log(entity,rr.LineStrips3D.from_fields(colors=color,
                   radii=rr.Radius.ui_points(1.5)),static=True)
            seg=np.stack([p[:-1],p[1:]],axis=1)[ok]
            rr.send_columns(entity,indexes=[rr.TimeColumn('seconds',duration=(t[1:][ok]).astype('timedelta64[ns]'))],
                            columns=rr.LineStrips3D.columns(strips=seg))
            rr.log(entity+'/samples',rr.Points3D.from_fields(colors=color,
                   radii=rr.Radius.ui_points(1.3)),static=True)
            rr.send_columns(entity+'/samples',indexes=[rr.TimeColumn('seconds',duration=t.astype('timedelta64[ns]'))],
                            columns=rr.Points3D.columns(positions=p))
            chosen=(t>=start*1e9)&(t<=end*1e9)
            for axes,indices in [('XY',(0,1)),('XZ',(0,2))]:
                # Negate vertical display axis so north/up appears upward in a 2D view.
                pp=p[chosen][:,indices]*[1,-1]
                ent=f'/selected/{axes}/{mode}/{label}'
                rr.log(ent,rr.LineStrips2D(strips(t[chosen],pp,gap),colors=color,
                       radii=rr.Radius.ui_points(1.4)),static=True)
                rr.log(ent+'/samples',rr.Points2D(pp,colors=color,
                       radii=rr.Radius.ui_points(1.7 if role!='reference' else .5)),static=True)
            counts[mode+'_'+role]={'points':len(p),'temporal_segments':int(ok.sum())}
        for series,col in [('fusion',COLORS['fusion_online']),('reference',COLORS['reference'])]:
            entity=f'/steps/{mode}/{series}'
            rr.log(entity,rr.SeriesLines(colors=col,names=series+' displacement / 100ms'),static=True)
            rr.send_columns(entity,indexes=[rr.TimeColumn('seconds',duration=z[mode+'_step_t_ns'].astype('timedelta64[ns]'))],
                            columns=rr.Scalars.columns(scalars=z[mode+'_'+series+'_step_m']))
    with (H/'render_counts.json').open('x') as f:
        json.dump({'rerun_version':rr.__version__,'counts':counts},f,indent=2)
    print(dest)


if __name__=='__main__':
    main()
