"""Roomier full-route camera and lighter cloud rendering, no geometry edits."""
from pathlib import Path
import subprocess
import numpy as np
import rerun as rr
import rerun.blueprint as b
H=Path(__file__).resolve().parent
z=np.load(H.parent/'trajectories.npz')
p=np.concatenate([z[k] for k in z.files if k.endswith('_p')])
lo,hi=p.min(0),p.max(0);center=(lo+hi)/2;extent=float(np.max(hi-lo))
eye=b.EyeControls3D(kind=b.Eye3DKind.Orbital,position=center+[0,-.5*extent,1.25*extent],
                    look_target=center,eye_up=[0,0,1])
views=[]
for mode in ('off','on'):
    views.append(b.Spatial3DView(origin='/scene',contents=['+ /scene/environment/**',f'+ /scene/{mode}/**'],
        name='CBS '+mode.upper()+' | FOUR FULL TRAJECTORIES + VELODYNE',eye_controls=eye,
        overrides={'/scene/environment/Velodyne_saved_ON_map':[
            rr.Points3D.from_fields(radii=rr.Radius.ui_points(.35))]},
        line_grid=b.LineGrid3D(visible=True,spacing=1,stroke_width=.5,color=[65,75,78]),
        background=[16,20,23]))
blueprint=b.Blueprint(b.Vertical(
    b.Tabs(*views,b.TextDocumentView(origin='/notes',name='Reading the overlay'),active_tab=0),
    b.TextDocumentView(origin='/legend',name='Full paths | same coordinate frame | no smoothing'),row_shares=[.92,.08]),
    b.BlueprintPanel(state='collapsed'),b.SelectionPanel(state='collapsed'),b.TimePanel(state='hidden'),
    auto_layout=False,auto_views=False)
rbl=H/'full_route_layout.rbl';out=H/'leica2_full_overlay.rrd'
assert not rbl.exists() and not out.exists()
blueprint.save('LEICA-2 | full four-path overlay + Velodyne',rbl)
subprocess.run(['/home/yeranis/.local/bin/rerun','rrd','merge','-o',str(out),
    str(H/'leica2_full_paths_with_velodyne.rrd'),str(rbl)],check=True)
print(out)
