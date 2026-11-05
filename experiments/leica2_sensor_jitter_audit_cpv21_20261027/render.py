"""Render saved CP-V21 arrays with common axes and no clipped cohort values."""
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

H=Path(__file__).resolve().parent
out=H/'hesai_velodyne_off_timing_full_range.png'
assert not out.exists()
d=np.load(H/'timeseries.npz')
fig,axs=plt.subplots(3,2,figsize=(13,10),constrained_layout=True)
limits=[0.,0.]
for col,name in enumerate(('hesai','velodyne')):
    t=d[name+'_time_s']; op=d[name+'_online']; fp=d[name+'_finalized']; gp=d[name+'_reference']
    front=d[name+'_frontier_age_s']
    for p,color,label in [(op,'#d43f57','ONLINE'),(fp,'#17865c','DELAYED finalized'),(gp,'#347ac8','IE-TC')]:
        step=np.linalg.norm(np.diff(p,axis=0),axis=1)*100
        axs[0,col].plot(t[1:],step,color=color,lw=.8,label=label)
        limits[0]=max(limits[0],step[(t[1:]>0)&(t[1:]<=60)].max())
    axs[0,col].set(title=name.upper()+' CBS OFF',ylabel='100 ms step length (cm)',xlim=(0,60),xlabel='Seconds from capture start')
    axs[0,col].legend()
    for label,color,i in [('GLIM','#17865c',0),('Kimera','#dc921e',1)]:
        axs[1,col].plot(t,front[:,i]*1000,color=color,lw=.8,label=label)
    axs[1,col].set(ylabel='Newest received endpoint age (ms)',xlim=(10,12),ylim=(0,250),xlabel='Seconds from capture start (10–12 s detail)')
    axs[1,col].legend()
    correction=np.linalg.norm(op-fp,axis=1)*100
    axs[2,col].plot(t,correction,color='#d43f57',lw=.8)
    limits[1]=max(limits[1],correction[(t>0)&(t<=60)].max())
    axs[2,col].set(ylabel='Later correction magnitude (cm)',xlabel='Seconds from capture start',xlim=(0,60))
    for ax in axs[:,col]: ax.grid(alpha=.2)
for row,limit in ((0,limits[0]),(2,limits[1])):
    for ax in axs[row]: ax.set_ylim(0,limit*1.06)
fig.suptitle('LEICA-2: identical original receiver and physical output timestamps\nSaved CBS-OFF data; no smoothing or local fit. Endpoint age is not CPU latency.')
fig.savefig(out,dpi=150)
print({'figure':str(out),'unclipped_shared_maxima_cm':limits})
