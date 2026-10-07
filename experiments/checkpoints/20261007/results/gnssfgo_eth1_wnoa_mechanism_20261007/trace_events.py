from pathlib import Path
import csv,json
import numpy as np
from scipy.spatial.transform import Rotation
from prepare import ROOT
from beliefs import histories,old,stat
summary=json.loads((ROOT/'prediction_summary.json').read_text());out={};details=[]
for mode in ['off','on']:
    worst=summary[mode]['largest_current_correction'];a,b=worst['snapshot'],worst['next_snapshot']
    def ledger(label):return {(r['source'],int(r['from_ns']),int(r['to_ns'])):r for r in csv.DictReader((ROOT/(mode+'_wnoa')/('snapshot_'+label)/'ledger.csv').open())}
    before,after=ledger(a),ledger(b);h=histories(mode)
    for key,v in after.items():
        previous=before.get(key)
        if previous and previous['revision']==v['revision']:continue
        new=next(r for r in h[key] if r['revision']==int(v['revision']))
        ref=old.relative_reference(key[1],key[2]);e,_=old.discrepancy(new['mu'],ref)
        r=dict(mode=mode,source=key[0],from_ns=key[1],to_ns=key[2],kind='revision' if previous else 'new',new_revision=int(v['revision']),
               new_rho_reference_error_m=float(np.linalg.norm(e[3:])),new_phi_reference_error_deg=float(np.linalg.norm(e[:3])*180/np.pi),
               mean_rho_change_m='',mean_phi_change_deg='',rho_trace_ratio='',phi_trace_ratio='',old_rho_reference_error_m='',old_phi_reference_error_deg='')
        if previous:
            pre=next(r for r in h[key] if r['revision']==int(previous['revision']))
            refnew=(Rotation.from_rotvec(new['mu'][:3]),old.v_matrix(new['mu'][:3])@new['mu'][3:])
            change,_=old.discrepancy(pre['mu'],refnew);ep,_=old.discrepancy(pre['mu'],ref)
            r.update(mean_rho_change_m=float(np.linalg.norm(change[3:])),mean_phi_change_deg=float(np.linalg.norm(change[:3])*180/np.pi),
                     rho_trace_ratio=float(np.trace(new['cov'][3:,3:])/np.trace(pre['cov'][3:,3:])),phi_trace_ratio=float(np.trace(new['cov'][:3,:3])/np.trace(pre['cov'][:3,:3])),
                     old_rho_reference_error_m=float(np.linalg.norm(ep[3:])),old_phi_reference_error_deg=float(np.linalg.norm(ep[:3])*180/np.pi))
        details.append(r)
    out[mode]={}
    for source in ['glim_hesai','kimera_front']:
        rr=[r for r in details if r['mode']==mode and r['source']==source]
        out[mode][source]={'new':sum(r['kind']=='new' for r in rr),'revised':sum(r['kind']=='revision' for r in rr),
                         **{k:stat([r[k] for r in rr if r[k]!='']) for k in ['mean_rho_change_m','mean_phi_change_deg','rho_trace_ratio','phi_trace_ratio','old_rho_reference_error_m','new_rho_reference_error_m']}}
with (ROOT/'largest_event_beliefs.csv').open('x',newline='') as f:
    w=csv.DictWriter(f,fieldnames=list(details[0]));w.writeheader();w.writerows(details)
(ROOT/'largest_event_beliefs.json').write_text(json.dumps(out,indent=2))
