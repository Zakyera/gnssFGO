"""CP1: frozen numeric telemetry only; no estimator, bag or optimizer execution."""
import importlib.util
import json
import re
import sys
from pathlib import Path

import numpy as np

sys.dont_write_bytecode = True
HERE = Path(__file__).resolve().parent
PREV = HERE.parent / 'fixed_lag_covariance_fl4z_20261017'
spec = importlib.util.spec_from_file_location('fl4z', PREV / 'analyze.py')
a = importlib.util.module_from_spec(spec)
spec.loader.exec_module(a)
g = a.geo


def trace_logs(path):
    provider, decomposition, status, exports = {}, {}, {}, {}
    counts = dict(provider=0, first=0, revised=0, matrix_dumps=0)
    last_provider = None
    with path.open() as f:
        for raw in f:
            line = re.sub(r'\x1b\[[0-9;]*m', '', raw).strip()
            if 'SHARED_NATIVE_PROVIDER_QUERY ' in line:
                fields = dict(item.split('=', 1) for item in line.split('SHARED_NATIVE_PROVIDER_QUERY ', 1)[1].split())
                i, j = int(fields['from']) & ((1 << 56)-1), int(fields['to']) & ((1 << 56)-1)
                assert (i,j) not in provider
                for k in ['epoch','admitted','active','absorbed','removed','resets']:
                    fields[k] = int(fields[k])
                assert fields['admitted'] == fields['active']+fields['absorbed']+fields['removed']
                assert fields['resets'] == 0
                fields['cov'] = g.matrix(fields.pop('matrix'))
                provider[i,j] = fields
                last_provider = (i,j)
                counts['provider'] += 1
            elif 'CBS_ODOM_RELATIVE_COVARIANCE_DECOMP_ROW,' in line:
                f = line.split('CBS_ODOM_RELATIVE_COVARIANCE_DECOMP_ROW,',1)[1].split(',')
                assert f[:3] == ['G2K','g','k']
                key = (int(f[3][1:]),int(f[4][1:]))
                assert key == last_provider and key not in decomposition
                decomposition[key] = dict(zip(['joint_ff','joint_tt','prop_from','prop_to','prop_cross','prop_sum'],map(float,f[5:])))
            elif 'CBS_ODOM_RELATIVE_COVARIANCE_ROW,' in line:
                f = line.split('CBS_ODOM_RELATIVE_COVARIANCE_ROW,',1)[1].split(',')
                key = (int(f[3][1:]),int(f[4][1:]))
                assert key == last_provider and key not in status
                assert f[5] == 'schur_relative_between' and f[-1] == 'ok'
                assert len(f) == 25 and int(f[17]) == 6 and int(f[20]) == 0
                status[key] = dict(schur_vs_direct=float(f[19]),jitter=int(f[20]))
            elif 'CBS_ODOM_RELATIVE_COVARIANCE_MATRIX_ROW,' in line:
                counts['matrix_dumps'] += 1
            else:
                tags = [('GLIM_CBS_EXPORT_BELIEF_ROW', 'first'), ('GLIM_CBS_REVISION_BELIEF_ROW','revised')]
                for tag, kind in tags:
                    if tag+',' not in line:
                        continue
                    f = line.split(tag+',',1)[1].split(',')
                    assert len(f) == 10 and f[0] == 'g'
                    i,j,rev = map(int,f[1:4])
                    update = j if kind == 'first' else int(f[5])
                    key = (i,j,rev,update)
                    assert key not in exports
                    mu,C = g.matrix(f[8]).ravel(), g.matrix(f[9])
                    exports[key] = (mu,C)
                    counts[kind] += 1
                    if kind == 'first':
                        assert (i,j) == last_provider and np.array_equal(C,provider[i,j]['cov'])
                    break
    assert set(provider) == set(decomposition) == set(status)
    return provider,decomposition,status,exports,counts


def analyze(seq):
    oldpath = PREV / f'{seq}_results.json'
    old = json.loads(oldpath.read_text())
    cfg = a.CASES[seq]
    inputs = {str(oldpath):g.sha(oldpath)}
    records = [r for r in old['records'] if r['source']=='glim_hesai' and r['stage']=='first']
    versions, logs, result = {}, {}, dict(sequence=seq, modes={})
    for mode in ['off','on']:
        root = a.W/'runs'/cfg[mode]/'prepared'
        vv, first, _, _ = a.read_events(root,'live60',mode,cfg,inputs)
        versions[mode] = {key:val for key,val in vv.items() if key[0]=='glim_hesai'}
        log = next((root/f'live60_{mode}_source/ros_logs').glob('*/glim_rosnode-*-stdout.log'))
        inputs[str(log)] = g.sha(log)
        p,d,s,ex,counts = trace_logs(log)
        logs[mode] = (p,d,s)
        matched = 0
        for key,history in versions[mode].items():
            for x in history.values():
                export_key = (x['source_i'],x['source_j'],x['source_revision'],x['update'])
                mu,C = ex[export_key]
                assert np.array_equal(mu,x['mu']) and np.array_equal(C,x['cov'])
                matched += 1
        assert matched == len(ex)
        # Decomposition traces mix radians and metres; use only as an arithmetic
        # check/cancellation description, NEVER as a physical confidence scale.
        sum_error = []
        ratios = []
        for key,z in d.items():
            independent = z['prop_from']+z['prop_to']
            sum_error.append(abs(independent+z['prop_cross']-z['prop_sum'])/independent)
            ratios.append(-z['prop_cross']/independent)
            assert abs(z['prop_sum']-np.trace(p[key]['cov'])) < 2e-5*np.trace(p[key]['cov'])
        assert max(sum_error) < 2e-5 # six-significant-digit text diagnostics
        result['modes'][mode] = dict(log_counts=counts,exact_source_transport_matches=matched,
            first_provider_covariances_exact=True,
            mixed_unit_cross_cancellation_fraction=a.stat(ratios),
            decomposition_relative_sum_error=max(sum_error),
            schur_direct_max=max(x['schur_vs_direct'] for x in s.values()),
            schur_direct_relative_trace_max=max(s[k]['schur_vs_direct']/np.trace(p[k]['cov']) for k in s),
            provider_first_epoch=min(x['epoch'] for x in p.values()),
            provider_last={k:v for k,v in p[max(p)].items() if k not in ('cov','query_ms')})
        # Every read input already used by FL4z must still match that audit.
        for path,h in inputs.items():
            if path in old['input_hashes']:
                assert h == old['input_hashes'][path], path

    # Reference is recovered algebraically from the previously verified error:
    # Z_ref=Z_first Exp(e_first). No new GT interpolation or evaluator change.
    cohort, details = [], {}
    for r in records:
        key = ('glim_hesai',r['a'],r['b'])
        j = r['on_metadata']['source_j']
        if any(j+age not in versions[mode][key] for mode in ['off','on'] for age in [0,1,40]):
            continue
        targets = [g.exp(r[mode+'_mu'])@g.exp(np.r_[r[mode]['phi'],r[mode]['rho']]) for mode in ['off','on']]
        assert np.max(np.abs(targets[0]-targets[1])) < 1e-10
        row = dict(i=r['on_metadata']['source_i'],j=j,a=r['a'],b=r['b'])
        for mode in ['off','on']:
            row[mode] = {}
            for age in [0,1,40]:
                x = versions[mode][key][j+age]
                m = a.measure(x,None,targets[0])
                if age==0:
                    for label in ['rho_error_mm','rotation_error_deg','translation_trace_m2']:
                        assert abs(m[label]-r[mode][label]) < 1e-8
                row[mode][str(age)] = m
            row[mode]['first_revision_delay_s'] = (versions[mode][key][j+1]['arrival']-versions[mode][key][j]['arrival'])/1e9
            row[mode]['arrival_age40_s'] = (versions[mode][key][j+40]['arrival']-versions[mode][key][j]['arrival'])/1e9
        cohort.append(row)
        details[r['a'],r['b']] = row
    assert cohort
    result['matched_age40_cohort'] = dict(n=len(cohort), excluded_tail_or_missing=len(records)-len(cohort), modes={})
    for mode in ['off','on']:
        m = {}
        for age in ['0','1','40']:
            rr = [r[mode][age] for r in cohort]
            m[age] = {key:a.stat([x[key] for x in rr]) for key in ['translation_trace_m2','rotation_trace_rad2','translation_sigma_norm_mm','rotation_sigma_norm_deg','cartesian_error_mm','rho_error_mm','rotation_error_deg']}
        for age in ['1','40']:
            m['paired_'+age+'_over_first'] = {key:a.stat([r[mode][age][key]/r[mode]['0'][key] for r in cohort]) for key in ['translation_trace_m2','rotation_trace_rad2','rho_error_mm','rotation_error_deg']}
        m['first_revision_delay_s'] = a.stat([r[mode]['first_revision_delay_s'] for r in cohort])
        m['arrival_age40_s'] = a.stat([r[mode]['arrival_age40_s'] for r in cohort])
        result['matched_age40_cohort']['modes'][mode]=m

    sorted_r = sorted(records,key=lambda r:(r['on']['rho_error_mm'],r['a']))
    q = len(sorted_r)//4
    examples = [sorted_r[:q][q//2],sorted_r[-q:][q//2]]
    result['examples'] = []
    for label,r in zip(['lower_error_quartile_median','higher_error_quartile_median'],examples):
        item = dict(selection=label,i=r['on_metadata']['source_i'],j=r['on_metadata']['source_j'],a=r['a'],b=r['b'])
        for mode in ['off','on']:
            key = (item['i'],item['j'])
            p,d,s = logs[mode]
            item[mode] = dict(first=r[mode],first_covariance=r[mode+'_cov'],first_mu=r[mode+'_mu'],
                decomposition_mixed_units=d[key],provider={k:v for k,v in p[key].items() if k not in ('cov','query_ms')},
                arithmetic_status=s[key])
        item['age40_available'] = (r['a'],r['b']) in details
        if item['age40_available']:
            item['evolution'] = details[r['a'],r['b']]
        result['examples'].append(item)
    assert all(g.sha(Path(p))==h for p,h in inputs.items())
    result['input_hashes']=inputs
    result['inputs_unchanged']=True
    a.write(HERE/f'{seq}_evidence.json',result)
    print(seq, 'completed', {mode:result['modes'][mode]['log_counts'] for mode in ['off','on']},'age40',len(cohort),flush=True)


if __name__=='__main__':
    analyze(sys.argv[1])
