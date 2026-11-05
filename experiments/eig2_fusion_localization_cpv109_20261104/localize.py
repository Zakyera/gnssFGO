"""Read sealed errors and locate intervals; no EVO scoring/optimization."""
import hashlib,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from evo.tools import file_interface
H=Path(__file__).resolve().parent;S=H.parent/'eig2_split_imu_live_pair_cpv108_20261104'
P=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/eig2_split_imu_live_pair_cpv108_20261104/prepared')
PINS={}
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def pin(p):
    p=Path(p);PINS[str(p)]=sha(p);return p
def read(p):return json.loads(pin(p).read_text())
def save(p,x):
    with p.open('x') as f:json.dump(x,f,indent=2,allow_nan=False)
def stats(a):
    a=np.asarray(a);a=a[np.isfinite(a)]
    return dict(n=len(a),mean=float(a.mean()),rms=float(np.sqrt(np.mean(a*a))),median=float(np.median(a)),p95=float(np.percentile(a,95)),max=float(a.max()))
def runs(mask):return [a for a in np.split(np.flatnonzero(mask),np.flatnonzero(np.diff(np.flatnonzero(mask))>1)+1) if len(a)]
def rolling(a,valid):
    out=np.full(len(a),np.nan);width=51
    for ix in runs(valid):
        if len(ix)<width:continue
        out[ix[25:-25]]=np.sqrt(np.convolve(a[ix]**2,np.ones(width)/width,'valid'))
    return out
def separated_peaks(score,t,spacing=15,count=5):
    ids=[]
    for i in np.argsort(np.nan_to_num(score,nan=-np.inf))[::-1]:
        if not np.isfinite(score[i]):continue
        if all(abs(t[i]-t[j])>=spacing for j in ids):ids.append(int(i))
        if len(ids)==count:break
    return ids
def main():
    assert not (H/'localization.json').exists()
    sealed=read(S/'complete.json')['output_sha256'];r=read(S/'results.json');audit=read(P/'input_audit.json')
    z=np.load(pin(S/'evaluation/comparison.npz'));ts=z['timestamp_ns'];t=(ts-audit['epoch_ns'])*1e-9
    valid=z['common_valid'];gp=z['reference_position'];gr=z['reference_rotation']
    arrays=dict(time_s=t,valid=valid,reference_position=gp);out={};alignchecks={}
    for key,row in r['common'].items():
        a=row['alignment'];p=z[key+'_position'];rot=z[key+'_rotation']
        aligned=p@np.asarray(a['rotation']).T+np.asarray(a['translation']);err=aligned-gp
        norm=np.linalg.norm(err,axis=1);norm[~valid]=np.nan
        saved=file_interface.load_res_file(pin(S/'evaluation/common'/(key+'_ape.zip'))).np_arrays['error_array']
        alignchecks[key]=float(np.max(abs(norm[valid]-saved)))
        assert alignchecks[key]<1e-8 and abs(stats(norm)['rms']-row['ATE_RMSE_m'])<1e-12
        roll=rolling(norm,valid);local=np.full(len(t),np.nan);angle=local.copy()
        for ix in runs(valid):
            if len(ix)<=50:continue
            start,end=ix[:-50],ix[50:]
            dr=np.einsum('nji,nj->ni',rot[start],p[end]-p[start])
            dg=np.einsum('nji,nj->ni',gr[start],gp[end]-gp[start])
            local[end]=np.linalg.norm(dr-dg,axis=1)
            rr=rot[start].transpose(0,2,1)@rot[end];rg=gr[start].transpose(0,2,1)@gr[end]
            angle[end]=Rotation.from_matrix(rr.transpose(0,2,1)@rg).magnitude()*180/np.pi
        arrays.update({key+'_error':err,key+'_norm':norm,key+'_rms5':roll,key+'_relative5_m':local,key+'_relative5_deg':angle,key+'_aligned':aligned})
        out[key]=dict(error=stats(norm),peak_time_s=float(t[np.nanargmax(norm)]),relative5_m=stats(local),relative5_deg=stats(angle),axis_energy_fraction=(np.sum(err[valid]**2,axis=0)/np.sum(err[valid]**2)).tolist())
    fusion={}
    for role in ('fusion_online','fusion_delayed'):
        off=arrays['off_'+role+'_norm'];on=arrays['on_'+role+'_norm'];dif=on**2-off**2
        delta=arrays['on_'+role+'_rms5']-arrays['off_'+role+'_rms5'];alerts=[]
        for ids in runs(np.isfinite(delta)&(delta>.02)):
            if t[ids[-1]]-t[ids[0]]>=5:alerts.append(dict(start_s=float(t[ids[0]]),end_s=float(t[ids[-1]])))
        peaks=[]
        for i in separated_peaks(arrays['on_'+role+'_rms5'],t):
            peaks.append(dict(center_s=float(t[i]),start_s=float(t[i]-2.5),end_s=float(t[i]+2.5),on_rms_m=float(arrays['on_'+role+'_rms5'][i]),off_rms_m=float(arrays['off_'+role+'_rms5'][i]),extra_rms_m=float(delta[i])))
        bins=[]
        for lo in range(0,355,10):
            use=valid&(t>=lo)&(t<lo+10)
            if not use.any():continue
            bins.append(dict(start_s=lo,end_s=min(355.02,lo+10),n=int(use.sum()),off_rms_m=stats(off[use])['rms'],on_rms_m=stats(on[use])['rms'],net_extra_energy_fraction=float(np.sum(dif[use])/np.nansum(dif)),mean_error_off=arrays['off_'+role+'_error'][use].mean(axis=0).tolist(),mean_error_on=arrays['on_'+role+'_error'][use].mean(axis=0).tolist()))
        fusion[role]=dict(alerts=alerts,peaks=peaks,bins=bins,net_extra_squared_error=float(np.nansum(dif)))
    for p,digest in PINS.items():
        assert sha(p)==digest
        if p in sealed:assert digest==sealed[p]
    np.savez_compressed(H/'localized.npz',**arrays)
    result=dict(passed=True,epoch_ns=audit['epoch_ns'],rows=int(valid.sum()),evo_array_max_errors=alignchecks,curves=out,fusion=fusion,input_sha256=PINS,
                caveat='Global fit errors localize score contributions, not causal onset. Relative5 diagnostics use no fitted alignment; not0.5mEVO RPE.')
    save(H/'localization.json',result)
    print(json.dumps(dict(curves=out,fusion=fusion),indent=2))
if __name__=='__main__':main()
