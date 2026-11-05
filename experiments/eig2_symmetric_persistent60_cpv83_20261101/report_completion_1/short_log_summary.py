def log_summary(log,costpath):
    timing=[];covs=[];first=active=0
    with log.open() as f:
        for line in f:
            if 'KIMERA_OPTIMIZE_TIMING_ROW,' in line:
                timing.append([float(x) for x in m.ANSI.sub('',line.split('KIMERA_OPTIMIZE_TIMING_ROW,',1)[1]).strip().split(',')])
            if 'KIMERA_CBS_REVISION_EDGE_ROW,' in line:
                row=m.ANSI.sub('',line.split('KIMERA_CBS_REVISION_EDGE_ROW,',1)[1]).strip().split(',')
                assert len(row)==64;covs.append(np.array(row[28:],float).reshape(6,6))
                if row[7]=='first':first+=1
                elif row[7]=='active':active+=1
                else:raise AssertionError(row[7])
    tt=np.array(timing);cc=np.array(covs);cost=np.loadtxt(costpath,delimiter=',')
    assert np.isfinite(tt).all() and np.isfinite(cc).all() and np.isfinite(cost).all()
    assert np.array_equal(tt[:,0],np.arange(len(tt))) and np.all(tt[:,-1]==1)
    assert np.array_equal(cost[:,0],np.arange(len(cost)))
    sym=float(np.max(abs(cc-cc.transpose(0,2,1))));assert sym<1e-10
    eigen=np.linalg.eigvalsh(cc);assert eigen.min()>0
    return dict(optimize_ms=stats(tt[:,1]),smoother_ms=stats(tt[:,5]),rows=len(cost),post_solve_total_cost=stats(cost[:,5]),post_solve_imu_cost=stats(cost[:,7]),post_solve_imu_cost_after_250s=None,after_250s_note='Not applicable: this recording is 60s',native_prior_counter_max=int(cost[:,3].max()),covariance=dict(rows=len(cc),first=first,active=active,symmetry_max=sym,eigenvalue_min=float(eigen.min()),rot_trace_rad2=stats(np.trace(cc[:,:3,:3],axis1=1,axis2=2)),translation_trace_m2=stats(np.trace(cc[:,3:,3:],axis1=1,axis2=2)))),cost,tt
