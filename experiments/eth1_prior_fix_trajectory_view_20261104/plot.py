"""Display-only supplement: sealed CP104 results and saved alignment, no re-scoring."""
import datetime
import hashlib
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

H = Path(__file__).resolve().parent
S = H.parent/'eth1_prior_fix_evaluation_cpv104_20261104'
C = S/'evaluation/common'
def sha(path):
    with path.open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()
seal = json.loads((S/'complete.json').read_text())
assert seal['passed'] and not (H/'provenance.json').exists()
inputs = {}
def checked(path):
    digest = sha(path)
    assert seal['output_sha256'][str(path)] == digest, path
    inputs[str(path)] = digest
    return path
r = json.loads(checked(S/'results.json').read_text())
reference = np.loadtxt(checked(C/'reference.tum'))
assert reference.shape == (2966, 8)
cuts = np.split(np.arange(len(reference)), np.flatnonzero(np.diff(reference[:,0]) > .1000001)+1)
assert [len(c) for c in cuts] == [2873, 83, 10]
origin = reference[0,1:4]
ref = reference[:,1:4]-origin
series = {}
for mode in ('CP95_OFF', 'CP103_ON'):
    for role in ('glim', 'kimera', 'fusion_online'):
        key = mode+'_'+role
        values = np.loadtxt(checked(C/(key+'.tum')))
        assert np.array_equal(values[:,0], reference[:,0])
        a = r['common'][key]['alignment']
        assert a['scale'] == 1
        series[key] = values[:,1:4]@np.asarray(a['rotation']).T+np.asarray(a['translation'])-origin
plt.rcParams.update({'font.size':11, 'axes.spines.top':False, 'axes.spines.right':False})
fig, axes = plt.subplots(1,2,figsize=(13,7.3), sharex=True, sharey=True)
roles = [('glim','GLIM','#0072b2','-'), ('kimera','Kimera','#d55e00','--'),
         ('fusion_online','ONLINE fusion','#009e73','-')]
all_xy = np.concatenate([ref[:,:2]]+[s[:,:2] for s in series.values()])
lo,hi = all_xy.min(axis=0),all_xy.max(axis=0)
pad = np.maximum((hi-lo)*.06,1.)
for ax,mode,title in zip(axes,('CP95_OFF','CP103_ON'),
                         ('CBS OFF — saved live control', 'CBS ON — corrected live run')):
    for role,label,color,style in roles:
        p = series[mode+'_'+role]
        for i,ii in enumerate(cuts):
            ax.plot(p[ii,0],p[ii,1],color=color,ls=style,lw=1.25,alpha=.95,
                    label=label if i==0 else None)
    for i,ii in enumerate(cuts):
        ax.plot(ref[ii,0],ref[ii,1],color='#202020',lw=1.1,ls=':',
                label='IE-TC reference' if i==0 else None)
    ax.set(title=title,xlabel='Reference-frame X (m)',xlim=(lo[0]-pad[0],hi[0]+pad[0]),
           ylim=(lo[1]-pad[1],hi[1]+pad[1]))
    ax.set_aspect('equal', adjustable='box')
    ax.grid(alpha=.2)
axes[0].set_ylabel('Reference-frame Y (m)')
handles,labels = axes[0].get_legend_handles_labels()
fig.legend(handles,labels,loc='upper center',bbox_to_anchor=(.5,.9),ncol=4,frameon=False)
fig.suptitle('ETH-1 | GLIM, Kimera and ONLINE fusion trajectories',y=.97,fontsize=16)
fig.text(.5,.045,'Same 296.3 s supported coverage · SE3 alignment, no scale · Gaps are not connected',
         ha='center',fontsize=10)
fig.text(.5,.018,'OFF: saved CP95 control; ON: corrected CP103 backend. Not a newly matched same-binary pair.',
         ha='center',fontsize=9,color='#444444')
fig.subplots_adjust(left=.08,right=.98,bottom=.13,top=.81,wspace=.13)
for ext in ('png','svg'):
    dest=H/('eth1_off_on_trajectories.'+ext)
    assert not dest.exists()
    fig.savefig(dest,dpi=170,facecolor='white')
plt.close(fig)
for path,digest in inputs.items():
    assert sha(Path(path)) == digest
out=dict(passed=True,created=datetime.datetime.now().astimezone().isoformat(),
         source_complete_sha256=sha(S/'complete.json'),input_sha256=inputs,
         plot_script_sha256=sha(Path(__file__)),
         outputs={p.name:sha(p) for p in H.glob('eth1_off_on_trajectories.*')},
         plot='XY projection of saved SE3-aligned prism-point trajectories; common translation of view origin only',
         common_rows=2966,supported_s=296.3,segments=[len(c) for c in cuts],
         reference_segmented_path_m=r['common']['CP103_ON_glim']['reference_path_length_m'],
         old_outputs_modified=False,new_scoring=False,new_capture=False,
         note='OFF saved CP95 and corrected ON CP103. Different backend, not a new matched OFF/ON pair. DELAYED is not shown in this ONLINE plot.')
with (H/'provenance.json').open('x') as f:
    json.dump(out,f,indent=2)
print(json.dumps({k:v for k,v in out.items() if k not in ('input_sha256','outputs')},indent=2))
