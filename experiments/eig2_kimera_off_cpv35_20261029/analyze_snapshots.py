"""Read-only saved observer snapshots; no optimization or accuracy metric."""
import csv,json,sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
H=Path(__file__).resolve().parent
P=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/eig2_kimera_off_cpv35_20261029')
def states(path):return {r[0]:np.array(r[1:],float) for r in csv.reader(path.open())}
def rotation(v):return Rotation.from_quat(v[[4,5,6,3]])
def skew(x):return np.array([[0,-x[2],x[1]],[x[2],0,-x[0]],[-x[1],x[0],0]])
def local(k,a,b):
    if k[0]!='x':return b-a
    R=rotation(a);w=(R.inv()*rotation(b)).as_rotvec();t=R.inv().apply(b[:3]-a[:3]);s=np.linalg.norm(w);wx=skew(w)
    Jinv=np.eye(3)-.5*wx+(1/12 if s<1e-5 else (1-(s/2)/np.tan(s/2))/(s*s))*(wx@wx)
    return np.r_[w,Jinv@t]
def stats(a):
    return dict(max=float(np.max(a)),rms=float(np.sqrt(np.mean(np.square(a))))) if len(a) else dict(max=None,rms=None)
def stage(d):
    x=states(d/'state.csv');t=states(d/'theta.csv');fixed=(d/'fixed_keys.txt').read_text().splitlines()
    costs=list(csv.DictReader((d/'factor_costs.csv').open()));sums={}
    for c in costs:
        fam=c['family'];v=sums.setdefault(fam,dict(n=0,state=0.,theta=0.));v['n']+=1
        v['state']+=float(c['error_at_state']);v['theta']+=float(c['error_at_theta'])
    rows=list(csv.DictReader((d/'imu_residuals.csv').open()));errors={}
    for part,cols,scale in [('rotation_deg',['rotation_x','rotation_y','rotation_z'],180/np.pi),('position_m',['position_x','position_y','position_z'],1),('velocity_mps',['velocity_x','velocity_y','velocity_z'],1)]:
        vals=[np.linalg.norm([float(r[c]) for c in cols])*scale for r in rows];errors[part]=stats(vals)
        if vals:errors[part]['worst_slot']=int(rows[int(np.argmax(vals))]['slot'])
    poses=[k for k in x if k[0]=='x'];vkeys=[k for k in x if k[0]=='v'];bkeys=[k for k in x if k[0]=='b']
    priors=[]
    for file in sorted(d.glob('prior_*_augmented.csv')):
        slot=int(file.name.split('_')[1]);a=np.loadtxt(file,delimiter=',');keys=[r[0] for r in csv.reader((d/('prior_'+str(slot)+'_keys.csv')).open())]
        delta=np.concatenate([local(k,t[k],x[k]) for k in keys]);hh=(a[:-1,:-1]+a[:-1,:-1].T)*.5;b=a[:-1,-1];const=a[-1,-1]
        def energy(values):
            dd=np.concatenate([local(k,t[k],values[k]) for k in keys]);return float(.5*(dd@hh@dd-2*dd@b+const))
        calculated=energy(x);logged=float(next(c['error_at_state'] for c in costs if int(c['slot'])==slot))
        assert abs(calculated-logged)<1e-5+1e-8*abs(logged),(file,calculated,logged)
        # Finite common-world transforms rotate ALL pose/velocity prior keys.
        # This diagnoses the retained prior function, not a new live constraint.
        centroid=np.mean([x[k][:3] for k in poses],axis=0)
        sweep={}
        for mode in ('tx','ty','tz','yaw'):
            ee=[]
            for amount in (-.1,0,.1):
                rr=Rotation.from_rotvec([0,0,amount]) if mode=='yaw' else Rotation.identity();tt=np.zeros(3)
                if mode!='yaw':tt['xyz'.index(mode[1])]=amount
                moved={k:v.copy() for k,v in x.items()}
                for k in keys:
                    if k[0]=='x':
                        moved[k][:3]=centroid+rr.apply(x[k][:3]-centroid)+tt;q=(rr*rotation(x[k])).as_quat();moved[k][3:]=q[[3,0,1,2]]
                    elif k[0]=='v':moved[k]=rr.apply(x[k])
                ee.append(energy(moved))
            sweep[mode]=dict(step=.1,unit='rad' if mode=='yaw' else 'm',energies=ee,central_curvature=(ee[0]+ee[2]-2*ee[1])/.01)
        priors.append(dict(slot=slot,dimension=len(delta),pose_keys=sum(k[0]=='x' for k in keys),state_energy=logged,reconstructed_energy=calculated,energy_absolute_difference=abs(calculated-logged),gauge_sweep=sweep))
    return dict(poses=len(poses),fixed_count=len(fixed),fixed_pose_count=sum(k[0]=='x' for k in fixed),fixed_keys=fixed,
       anchor_rotation_deg=stats([np.linalg.norm(local(k,t[k],x[k])[:3])*180/np.pi for k in poses]),
       anchor_position_m=stats([np.linalg.norm(x[k][:3]-t[k][:3]) for k in poses]),
       anchor_velocity_mps=stats([np.linalg.norm(x[k]-t[k]) for k in vkeys]),
       velocity_norm_mps=stats([np.linalg.norm(x[k]) for k in vkeys]),
       accel_bias_norm_mps2=stats([np.linalg.norm(x[k][:3]) for k in bkeys]),
       gyro_bias_norm_radps=stats([np.linalg.norm(x[k][3:]) for k in bkeys]),
       family_costs=sums,imu_residuals=errors,priors=priors),x
def main():
    name=sys.argv[1];out=H/name;assert not out.exists()
    result=[]
    for d in sorted((P/'capture/snapshots').glob('update_*'),key=lambda d:int(d.name.split('_')[1])):
        if not (d/'after_marginalization/complete.txt').exists():continue
        item={'update':int(d.name.split('_')[1]),'stages':{}};xs={}
        for phase in ('before_update','after_solve','after_marginalization'):
            item['stages'][phase],xs[phase]=stage(d/phase)
        b=xs['before_update'];s=xs['after_solve'];m=xs['after_marginalization'];keys=[k for k in m if k in b and k[0]=='x']
        item['solve_pose_revision_m']=stats([np.linalg.norm(b[k][:3]-s[k][:3]) for k in keys])
        item['marginalization_max_state_component_change']=max((float(np.max(np.abs(s[k]-v))) for k,v in m.items()),default=0.)
        result.append(item)
    with out.open('x') as f:json.dump(dict(passed=True,updates=result,snapshot_only=True,not_accuracy_or_stationarity=True),f,indent=2)
    print(json.dumps(dict(updates=len(result),last_update=result[-1]['update'] if result else None,output=str(out)),indent=2))
if __name__=='__main__':main()
