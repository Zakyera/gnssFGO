"""Consolidate independent fixed-camera findings with explicit provenance."""
import collections
import importlib.util
import json
from pathlib import Path
import numpy as np

H=Path(__file__).resolve().parent
s=importlib.util.spec_from_file_location('cp57',H/'analyze.py')
m=importlib.util.module_from_spec(s);s.loader.exec_module(m)
s=importlib.util.spec_from_file_location('bc',H/'boundary_check.py')
bc=importlib.util.module_from_spec(s);s.loader.exec_module(bc)


def independent_metrics(row, point):
    """Direct camera-by-camera projection, independent of vectorized implementation."""
    errors=[]; depths=[]; distances=[]; cost=0.
    for c in row['cameras']:
        T=np.array(c['pose']).reshape(4,4);K=np.array(c['K']).reshape(3,3)
        local=T[:3,:3].T@(point-T[:3,3])
        uv=K@local;uv=uv[:2]/uv[2]
        z=c['z'];cost+=(uv[0]-z[0])**2+(uv[1]-z[2])**2
        for side,u in enumerate(z[:2]):
            if u is None:continue
            q=local-np.array([side*c['baseline'],0,0])
            pix=K@q;pix=pix[:2]/pix[2]
            errors.append(float(np.linalg.norm(pix-[u,z[2]])))
            depths.append(float(q[2]));distances.append(float(np.linalg.norm(q)))
            if side:cost+=(pix[0]-u)**2
    return max(errors),min(depths),max(distances),cost/18


def verify_point(row,candidate):
    p=np.array(candidate['point_world'])
    e,z,d,c=independent_metrics(row,p)
    assert abs(e-candidate['max_px'])<1e-7*(1+e)
    assert abs(c-candidate['smart_cost'])<1e-7*(1+c)
    assert abs(z-candidate['min_depth_m'])<1e-8*(1+abs(z))
    assert abs(d-candidate['max_distance_m'])<1e-8*(1+d)
    if candidate['passes_geometric_gates']:assert e<=3 and z>0 and d<=10


def main():
    sources=[H/'attempt_inner1/tracks.jsonl',H/'boundary_analysis.json',H/'convex_candidates.json',
             m.GEOMETRY,m.ACTIVITY,Path(__file__),H/'boundary_check.py',H/'analyze.py']
    pins={str(p):m.sha(p) for p in sources}
    rows=[json.loads(l) for l in sources[0].open()]
    boundary={(r['phase'],r['slot']):r for r in json.loads(sources[1].read_text())['results']}
    convex={(r['phase'],r['slot'],r['transform']):r for r in json.loads(sources[2].read_text())['results']}
    geometry={(r['phase'],r['slot'],r['transform']):r for line in m.GEOMETRY.open() if (r:=json.loads(line))}
    activity={(r['label'],r['slot']):r for line in m.ACTIVITY.open() if (r:=json.loads(line))}
    records=[];certificate_count=0;point_count=0;native_cost_error=[]
    for r in rows:
        key=(r['phase'],r['slot'],r['transform']);row=geometry[key]
        g=m.Geometry(row)
        for c in r['smart_L2']['starts']+r['mono_L2']['starts']+[r['native']]:
            verify_point(row,c);point_count+=1
        if r['transform']=='identity' and r['native_status']==0:
            delta=abs(r['native']['smart_cost']-activity[('on_'+r['phase'],r['slot'])]['cold_cost'])
            assert delta<1e-7*(1+r['native']['smart_cost']);native_cost_error.append(delta)
        f=r['feasibility'];candidate=f.get('candidate');cert=f.get('certificate')
        if cert:
            y=np.array(cert['nonnegative_weights']);A=np.array(cert['constraint_rows']);b=np.array(cert['constraint_rhs'])
            assert np.all(y>=0)
            margin=-y@b-10*np.sum(np.abs(y@A))
            assert margin>1e-8 and abs(margin-cert['margin'])<1e-8
            certificate_count+=1
        if candidate:verify_point(row,candidate);point_count+=1
        if key in convex and convex[key]['outcome']=='feasible':candidate=convex[key]['best']
        if r['transform']=='identity' and (r['phase'],r['slot']) in boundary:
            b=boundary[(r['phase'],r['slot'])]
            if b['best']['passes_geometric_gates']:candidate=b['best']
        if candidate and candidate['passes_geometric_gates']:
            verify_point(row,candidate);point_count+=1
            assert not cert
            outcome='feasible'
        else:outcome=f['outcome']
        assert outcome in ['feasible','infeasible'],key
        # Refit the ORIGINAL smart L2, constrained only by depth/distance,
        # from a verified feasible point. Never constrain reprojection to keep
        # a track alive or treat minimax as the native statistical objective.
        l2=r['smart_L2']['best']
        if r['transform']=='identity' and r['phase']!='ordinary_theta' and r['warm_status']!=0 and outcome=='feasible':
            constrained=bc.optimize(g,np.array(candidate['point_world'])-g.origin,False)
            verify_point(row,constrained);point_count+=1
            if (l2['min_depth_m']<=0 or l2['max_distance_m']>10 or constrained['smart_cost']<l2['smart_cost']) and constrained['min_depth_m']>0 and constrained['max_distance_m']<=10:
                l2=constrained
        record=dict(slot=r['slot'],phase=r['phase'],transform=r['transform'],
                    initial_warm_status=r['initial_warm_status'],warm_status=r['warm_status'],
                    native_status=r['native_status'],native=r['native'],rank=r['rank'],
                    in_original_27=r['in_original_27'],stereo=r['stereo'],keys=r['keys'],
                    geometry_hash=r['geometry_hash'],L2=l2,outcome=outcome,
                    feasible_candidate=candidate if outcome=='feasible' else None,
                    certificate=cert)
        records.append(record)
    counts={}
    for phase in ['ordinary_estimate','refresh_estimate']:
        selected=[r for r in records if r['phase']==phase and r['transform']=='identity'
                  and r['initial_warm_status']==0 and r['warm_status']!=0]
        rejected=[r for r in selected if r['native_status']==3]
        counts[phase]=dict(total_lost=len(selected),rank_rejected=sum(r['native_status']==1 for r in selected),
            outlier_instances=len(rejected),distinct_outlier_geometries=len({r['geometry_hash'] for r in rejected}),
            geometric_feasible_outliers=sum(r['outcome']=='feasible' for r in rejected),
            geometric_infeasible_outliers=sum(r['outcome']=='infeasible' for r in rejected),
            L2_recovered_outliers=sum(r['L2']['passes_geometric_gates'] for r in rejected),
            L2_recovered_distinct=len({r['geometry_hash'] for r in rejected if r['L2']['passes_geometric_gates']}),
            feasible_distinct=len({r['geometry_hash'] for r in rejected if r['outcome']=='feasible'}),
            infeasible_distinct=len({r['geometry_hash'] for r in rejected if r['outcome']=='infeasible'}),
            lost_rank_geometry_feasible=sum(r['native_status']==1 and r['outcome']=='feasible' for r in selected))
    # Native origin shifts are artificial. Quantify their relation to REAL lost
    # tracks separately; do not call native translation reclassification a fix.
    gauge={}
    for phase in ['ordinary_theta','ordinary_estimate','refresh_estimate']:
        orig={r['slot']:r for r in records if r['phase']==phase and r['transform']=='identity'}
        for transform in ['rotation','translation']:
            transformed=[r for r in records if r['phase']==phase and r['transform']==transform]
            gauge[phase+'_'+transform]=dict(
                native_status_changes=[r['slot'] for r in transformed if r['native_status']!=orig[r['slot']]['native_status']],
                feasibility_status_changes=[r['slot'] for r in transformed if r['outcome']!=orig[r['slot']]['outcome']],
                actually_rejected_now_native_valid=[r['slot'] for r in transformed if orig[r['slot']]['warm_status']!=0 and r['native_status']==0])
            assert not gauge[phase+'_'+transform]['feasibility_status_changes']
    for p,h in pins.items():assert m.sha(p)==h,p
    result=dict(completed=True,inputs=pins,records=records,counts=counts,gauge=gauge,
                independent_point_checks=point_count,verified_certificates=certificate_count,
                native_smart_cost_comparisons=len(native_cost_error),native_smart_cost_max_error=max(native_cost_error),
                no_unresolved_geometry=True,graph_optimizer_calls=0,source_replays=0,production_changes=0)
    m.save('findings.json',result)
    print(json.dumps({k:v for k,v in result.items() if k not in ['records','inputs']},indent=2))


if __name__=='__main__':main()
