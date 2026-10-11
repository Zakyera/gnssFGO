"""Strict log contract for this capture; not an estimator/admission policy."""
import math
import re


def numbers(token):
    values = tuple(float(x) for x in re.split('[:;]', token))
    assert all(math.isfinite(x) for x in values), 'nonfinite payload'
    return values


def kimera_outcomes(lines):
    """Bind each construction/drop to its actual update and queued source edge.

    This capture has one candidate per unique receiver edge. Refuse ambiguous
    repeats rather than match a rejection from an unrelated historical update.
    """
    queued, outcomes = {}, {}
    update = None
    for ln, line in enumerate(lines, 1):
        if 'CBS_BPSAM_SOURCE_TRANSPORT_ROW,G2K,' in line:
            v = line.split('CBS_BPSAM_SOURCE_TRANSPORT_ROW,', 1)[1].strip().split(',')
            edge = (int(v[3][1:]), int(v[4][1:]))
            assert edge not in queued, ('duplicate transport', edge, ln)
            queued[edge] = dict(sender=(int(v[1][2:]), int(v[2][2:])),
                                cov=numbers(v[5]), converted=numbers(v[6]), line=ln)
        if 'CBS_BPSAM_ACTIVE_WINDOW_LIFECYCLE_ROW,k,' in line:
            v = line.split('CBS_BPSAM_ACTIVE_WINDOW_LIFECYCLE_ROW,', 1)[1].split(',')
            epoch = float(v[1])
            assert epoch.is_integer()
            update = int(epoch)
        result = None
        if 'CBS_ODOM_FACTOR_MATRIX_ROW,G2K,' in line:
            v = line.split('CBS_ODOM_FACTOR_MATRIX_ROW,', 1)[1].strip().split(',')
            edge = (int(v[1][1:]), int(v[2][1:]))
            result = dict(outcome='constructed', factor_cov=numbers(v[4]))
        if 'action=drop_before_stage' in line:
            m = re.search(r'CBS_BPSAM_ACTIVE_WINDOW_RECEIVER_EDGE_ROW,k,g,x(\d+)->x(\d+),active_window_pre_residual_observed,pre_residual=([^,]+),threshold=([^,]+),action=drop_before_stage', line)
            assert m, ('unexpected guard format', ln)
            edge = tuple(map(int, m.groups()[:2]))
            residual, threshold = map(float, m.groups()[2:])
            assert threshold == 2.0 and (not math.isfinite(residual) or residual > threshold)
            result = dict(outcome='drop_before_stage', pre_residual=residual, threshold=threshold)
        if result is not None:
            assert update is not None and edge in queued, ('no update/transport', edge, ln)
            key = (update, *edge)
            assert key not in outcomes, ('duplicate/contradictory outcome', key, ln)
            assert not any(k[1:] == edge for k in outcomes), ('ambiguous repeated receiver edge', edge)
            q = queued[edge]
            assert q['line'] < ln
            outcomes[key] = dict(result, **q, outcome_line=ln)
    assert len(queued) == len(outcomes), 'unaccounted queued candidates'
    return outcomes


def verify_candidates(candidates, outcomes, wire):
    used, result = set(), []
    for c in candidates:
        key = (c['update'], *c['receiver'])
        assert key not in used, ('duplicate alignment', key)
        used.add(key)
        assert key in outcomes, ('unexplained missing construction/guard', key)
        o = outcomes[key]
        assert c['sender'] == o['sender'], ('sender mismatch', key)
        assert len(c['cov']) == 36 and len(c['raw']) == len(c['aligned']) == 6
        assert len(c['times']) == 5 and max(c['times'][:4]) <= c['times'][4]
        assert all(math.isfinite(x) for x in (*c['cov'], *c['raw'], *c['aligned'], *c['times']))
        assert any(c['cov'] == p['cov'] and max(abs(x-y) for x,y in zip(c['raw'],p['mu'])) < 1e-12
                   and abs(c['times'][0]-p['ti']) <= 1e-6 and abs(c['times'][1]-p['tj']) <= 1e-6
                   for p in wire[c['sender']]), ('wire mismatch', key)
        assert c['cov'] == o['cov'] == o['converted'], ('transport covariance mismatch', key)
        if o['outcome'] == 'constructed':
            assert c['cov'] == o['factor_cov'], ('factor covariance mismatch', key)
        else:
            assert o['outcome'] == 'drop_before_stage'
        result.append(dict(update=c['update'], sender=c['sender'], receiver=c['receiver'],
                           times=c['times'], outcome=o['outcome'], outcome_line=o['outcome_line']))
    assert used == set(outcomes), 'unmatched outcome evidence'
    return result
