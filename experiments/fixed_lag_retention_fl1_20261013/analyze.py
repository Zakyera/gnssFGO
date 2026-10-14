"""FL1 metadata only. No GTSAM/ROS import, factor evaluation, or estimator call."""
import argparse
from collections import Counter, defaultdict
import csv
from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
import re

NS = 1_000_000_000
PERIOD = 100_000_000
REPO = Path('/home/yeranis/repos/gnssFGO')
CAPTURE = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_shared_linearization_s5_live160_20261012/prepared')
JOURNAL = CAPTURE / 'capacity160_on_receiver/events.csv'
ORACLE = Path('/home/yeranis/results/gnssfgo_endpoint_reuse_e2b_20261013/full_on1')
CODE = REPO / 'experiments/receiver_endpoint_reuse_e2b_20261013'
SOURCE_LOGS = {
    'glim_hesai': CAPTURE / 'capacity160_on_source/ros_logs/7a11af54-c67f-11f1-b007-362a2d7902c5/glim_rosnode-1-stdout.log',
    'kimera_front': CAPTURE / 'capacity160_on_source/estimators.log',
}


def require(condition, message):
    if not condition:
        raise ValueError(message)


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda: f.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def rows(path):
    with path.open(newline='') as f:
        return list(csv.DictReader(f))


def save_json(path, value):
    with path.open('x') as f:
        json.dump(value, f, indent=2, sort_keys=True)
        f.write('\n')


def save_csv(path, records):
    if not records:
        return
    with path.open('x', newline='') as f:
        fields = list(dict.fromkeys(k for record in records for k in record))
        writer = csv.DictWriter(f, fields)
        writer.writeheader()
        writer.writerows(records)


def binding(stamp, origin, period):
    require(stamp >= origin, 'before_initialization')
    left, remainder = divmod(stamp - origin, period)
    return (left,) if remainder == 0 else (left, left + 1)


def choose_lag(max_age_ns):
    # Predeclared 1 s margin; integer ceil to whole seconds.
    return ((max_age_ns + NS + NS - 1) // NS) * NS


def quantiles(values, divisor=NS):
    vals = sorted(values)
    if not vals:
        return {'count': 0}
    def q(frac):
        position = (len(vals) - 1) * frac
        i = int(position)
        return (vals[i] + (vals[min(i + 1, len(vals) - 1)] - vals[i]) * (position - i)) / divisor
    return dict(count=len(vals), minimum=vals[0] / divisor,
                median=q(.5), p95=q(.95), maximum=vals[-1] / divisor,
                mean=sum(vals) / len(vals) / divisor)


@dataclass(frozen=True)
class Belief:
    line: int
    arrival: int
    source: str
    session: str
    ordinal: int
    revision: int
    i: int
    j: int
    ti: int
    tj: int
    payload: str

    @property
    def identity(self):
        return (self.source, self.session, self.i, self.j)


class JournalModel:
    """Mirror only EventStream, CausalSupportClock and streaming dispatch order."""
    def __init__(self, startup=59, period=PERIOD):
        self.startup, self.period = startup, period
        self.clock, self.dispatch, self.excluded, self.progress, self.ops = [], [], [], [], []
        self.pending, self.buffered, self.ledger = [], [], {}
        self.beliefs = []
        self.imu_count, self.created, self.previous_physical = 0, -1, None
        self.previous_arrival, self.clock_available = -1, -1
        self.next_stamp, self.origin, self.end = None, None, None
        self.event_counts = Counter()
        self.queued = 0
        self.max_pending = 0

    def locate(self, b):
        a = binding(b.ti, self.origin, self.period)
        z = binding(b.tj, self.origin, self.period)
        return a, z, min(a + z), max(a + z)

    def exclude(self, b, reason):
        self.excluded.append(dict(source=b.source, session=b.session, ordinal=b.ordinal,
                                  revision=b.revision, from_timestamp_ns=b.ti,
                                  to_timestamp_ns=b.tj, reason=reason))

    def belief(self, b):
        if b.ti < self.origin:
            self.exclude(b, 'before_initialization')
            return
        *_, activation = self.locate(b)
        if activation > self.created:
            self.pending.append(b)
            self.ops.append(('pending', b))
            self.queued += 1
            self.max_pending = max(self.max_pending, len(self.pending))
        else:
            self.insert(b, b.arrival)

    def insert(self, b, insertion):
        a, z, oldest, required = self.locate(b)
        require(required <= self.created, 'missing support at dispatch')
        require(insertion >= max(b.arrival, self.clock[required]['available_timestamp_ns']), 'noncausal dispatch')
        old = self.ledger.get(b.identity)
        outcome = 'insert' if old is None else ('duplicate' if b.revision == old.revision else
                                               'stale' if b.revision < old.revision else 'replace')
        if old is not None:
            require((b.ti, b.tj) == (old.ti, old.tj), 'stable ID endpoint change')
            # Raw-equal payload guard is stricter than C++'s 1e-12 tolerance.
            # No duplicates exist in the audited capture; fail closed otherwise.
            if outcome == 'duplicate':
                require(b.payload == old.payload, 'nonidentical duplicate requires separate numeric validation')
        record = dict(source=b.source, session=b.session, from_index=b.i, to_index=b.j,
                      revision=b.revision, arrival_ns=b.arrival, insertion_ns=insertion,
                      required_support=required, created_support=self.created, outcome=outcome)
        self.dispatch.append(record)
        self.ops.append(('dispatch', (b, record, oldest)))
        if outcome in ('insert', 'replace'):
            self.ledger[b.identity] = b

    def advance(self):
        while self.created + 1 < len(self.clock):
            self.created += 1
            self.ops.append(('create', self.created))
            if self.created == 0:
                for b in self.buffered:
                    self.belief(b)
                self.buffered.clear()
            for b in list(self.pending):
                if self.locate(b)[3] <= self.created:
                    self.insert(b, self.clock[self.created]['available_timestamp_ns'])
                    self.pending.remove(b)
            self.ops.append(('retire', self.created))

    def read(self, records):
        for line, f in enumerate(records, 1):
            require(self.end is None, 'event after E marker')
            kind, arrival = f[0], int(f[1])
            require(arrival >= self.previous_arrival, 'arrival moved backwards')
            self.previous_arrival = arrival
            self.event_counts[kind] += 1
            initialized_before = self.origin is not None
            if kind == 'E':
                require(len(f) == 2, 'invalid end schema')
                self.end = arrival
                break
            if kind == 'I':
                require(len(f) == 9, 'invalid clock schema')
                physical = int(f[2])
                require(arrival >= physical, 'clock availability precedes physical time')
                require(self.previous_physical is None or physical > self.previous_physical, 'clock moved backwards')
                self.previous_physical = physical
                self.clock_available = max(self.clock_available, arrival)
                self.imu_count += 1
                if self.imu_count == self.startup:
                    self.origin = self.next_stamp = physical
                while self.next_stamp is not None and self.next_stamp <= physical:
                    self.clock.append(dict(state_index=len(self.clock), physical_timestamp_ns=self.next_stamp,
                                           available_timestamp_ns=self.clock_available))
                    self.next_stamp += self.period
            else:
                require(kind == 'B' and len(f) == 52, 'invalid belief schema')
                require(all(re.fullmatch(r'[a-zA-Z0-9_-]+', x) for x in f[2:4]), 'invalid ID')
                source = {'g': 'glim', 'k': 'kimera'}.get(f[2], f[2])
                b = Belief(line, arrival, source, f[3], *map(int, f[4:10]),
                           hashlib.sha256(','.join(f[10:]).encode()).hexdigest())
                require(b.ti < b.tj <= arrival, 'invalid endpoint chronology')
                self.beliefs.append(b)
                if self.origin is None:
                    self.buffered.append(b)
                else:
                    self.belief(b)
            if initialized_before:
                self.progress.append(dict(event_count=line, arrival_ns=arrival,
                                          created_support=self.created, pending=len(self.pending)))
            self.advance()
        require(self.end is not None, 'missing E marker')
        require(not list(records), 'events after E marker')
        require(self.origin is not None, 'end before initialization')
        self.pending_at_end = len(self.pending)
        for b in self.pending:
            self.exclude(b, 'missing_right_support_at_end')
        return self


def reconcile(expected, actual, label):
    require(len(expected) == len(actual), f'{label} count: {len(expected)} != {len(actual)}')
    for n, (want, got) in enumerate(zip(expected, actual), 1):
        require(all(str(want[k]) == str(got[k]) for k in got), f'{label} row {n}: {want} != {got}')


def simulate(model, lag):
    active, absorbed, pending = {}, {}, {}
    boundary_rows = []
    minimum_kept, current = 0, -1
    counts = Counter()
    peak_before, peak_after = Counter(), Counter()
    first_retirement = None
    source_names = sorted({b.source for b in model.ledger.values()})

    def measure(target):
        support_count = current - minimum_kept + 1
        sources = Counter(b.source for b, _ in active.values())
        vals = dict(supports=support_count, variable_blocks=3 * support_count,
                    scalar_dimension=12 * support_count, explicit_source_factors=len(active),
                    explicit_motion_factors=max(0, support_count - 1), pending_versions=len(pending),
                    explicit_factors_excluding_marginals=len(active) + max(0, support_count - 1) +
                    (3 if minimum_kept == 0 else 0))
        vals.update({f'source_{s}': sources[s] for s in source_names})
        for k, v in vals.items():
            target[k] = max(target[k], v)
        return vals

    for op, value in model.ops:
        if op == 'create':
            current = value
        elif op == 'pending':
            pending[value.line] = value
        elif op == 'dispatch':
            b, record, oldest = value
            require(b.identity not in absorbed, f'revision after absorption: {b.identity}, r{b.revision}')
            require(oldest >= minimum_kept, f'retired bracket at dispatch: {b.identity}')
            require(record['created_support'] == current, 'operation order mismatch')
            pending.pop(b.line, None)
            previous = active.get(b.identity)
            expected = 'insert' if previous is None else ('replace' if b.revision > previous[0].revision else
                        'duplicate' if b.revision == previous[0].revision else 'stale')
            require(expected == record['outcome'], 'identity lifecycle mismatch')
            counts[expected] += 1
            if expected in ('insert', 'replace'):
                active[b.identity] = (b, oldest)
        elif op == 'retire':
            measure(peak_before)
            # Strictly older than c-L; equality is retained (ceil cutoff index).
            new_minimum = max(0, (current * model.period - lag + model.period - 1) // model.period)
            require(new_minimum >= minimum_kept, 'backward retirement')
            for b in pending.values():
                require(model.locate(b)[2] >= new_minimum, f'pending old bracket would retire: {b.identity}')
            consumed = [(identity, b, oldest) for identity, (b, oldest) in active.items()
                        if oldest < new_minimum]
            for identity, b, oldest in consumed:
                absorbed[identity] = dict(source=b.source, session=b.session, from_index=b.i,
                                         to_index=b.j, last_revision=b.revision, last_arrival_ns=b.arrival,
                                         retirement_support=current, oldest_support=oldest)
                del active[identity]
            if new_minimum > minimum_kept:
                if first_retirement is None:
                    first_retirement = current
                counts['retirement_boundaries'] += 1
            minimum_kept = new_minimum
            vals = measure(peak_after)
            boundary_rows.append(dict(current_support=current, oldest_retained_support=minimum_kept,
                                      absorbed_ids=len(absorbed), **vals))
        measure(peak_before)
    require(len(active) + len(absorbed) == len(model.ledger), 'lost or doubled stable ID')
    return dict(lag_ns=lag, dispatch_counts=dict(counts), active_ids_at_end=len(active),
                absorbed_ids_at_end=len(absorbed), finalized_supports=minimum_kept,
                remaining_supports=model.created - minimum_kept + 1,
                first_retirement_support=first_retirement,
                first_retirement_seconds=None if first_retirement is None else first_retirement * model.period / NS,
                peak_before_retirement=dict(peak_before), peak_after_retirement=dict(peak_after),
                lag_losses=0, pending_at_end=len(pending),
                active_by_source=dict(Counter(b.source for b, _ in active.values())),
                absorbed_by_source=dict(Counter(x['source'] for x in absorbed.values()))), boundary_rows, list(absorbed.values())


def analyze(model):
    versions, by_id = [], defaultdict(list)
    for op, value in model.ops:
        if op != 'dispatch':
            continue
        b, row, oldest = value
        if row['outcome'] not in ('insert', 'replace'):
            continue
        a, z, _, _ = model.locate(b)
        v = dict(journal_line=b.line, **row, from_timestamp_ns=b.ti, to_timestamp_ns=b.tj,
                 oldest_support=oldest, from_brackets=':'.join(map(str, a)), to_brackets=':'.join(map(str, z)),
                 required_age_ns=(row['created_support'] - oldest) * model.period,
                 arrival_endpoint_age_ns=b.arrival - b.tj,
                 support_wait_ns=row['insertion_ns'] - b.arrival,
                 interval_ns=b.tj - b.ti)
        versions.append(v)
        by_id[b.identity].append(v)
    max_age = max(v['required_age_ns'] for v in versions)
    groups = {}
    for source in ['all'] + sorted({v['source'] for v in versions}):
        group = [v for v in versions if source == 'all' or v['source'] == source]
        ids = [items for identity, items in by_id.items() if source == 'all' or identity[0] == source]
        groups[source] = dict(
            versions=len(group), ids=len(ids),
            dispatched_oldest_support_age_s=quantiles([v['required_age_ns'] for v in group]),
            first_dispatch_oldest_support_age_s=quantiles([v[0]['required_age_ns'] for v in ids]),
            last_observed_dispatch_oldest_support_age_s=quantiles([v[-1]['required_age_ns'] for v in ids]),
            first_arrival_minus_endpoint_s=quantiles([v[0]['arrival_endpoint_age_ns'] for v in ids]),
            last_observed_arrival_minus_endpoint_s=quantiles([v[-1]['arrival_endpoint_age_ns'] for v in ids]),
            first_to_last_observed_arrival_s=quantiles([v[-1]['arrival_ns'] - v[0]['arrival_ns'] for v in ids]),
            replacements_per_id=quantiles([len(v) - 1 for v in ids], 1),
            edge_duration_s=quantiles([v[0]['interval_ns'] for v in ids]),
            pending_support_wait_s=quantiles([v['support_wait_ns'] for v in group if v['support_wait_ns'] > 0]),
            worst_required_age=max(group, key=lambda v: v['required_age_ns']),
            last_five_seconds_ids=sum(v[-1]['arrival_ns'] >= model.end - 5 * NS for v in ids))
    lifecycle, boundaries, absorbed = simulate(model, choose_lag(max_age))
    identities = []
    for identity, v in by_id.items():
        identities.append(dict(source=identity[0], session=identity[1], from_index=identity[2], to_index=identity[3],
                               first_revision=v[0]['revision'], last_observed_revision=v[-1]['revision'],
                               versions=len(v), first_arrival_ns=v[0]['arrival_ns'], last_observed_arrival_ns=v[-1]['arrival_ns'],
                               last_support_age_ns=v[-1]['required_age_ns']))
    return dict(groups=groups, lifecycle=lifecycle, max_required_age_ns=max_age,
                margin_ns=NS, recommended_lag_ns=choose_lag(max_age),
                actual_headroom_ns=choose_lag(max_age) - max_age), versions, identities, boundaries, absorbed


def terminal_audit(model):
    """Source terminal markers are evidence only; not live transport events."""
    latest_receipt = {}
    by_line = {b.line: b for b in model.beliefs}
    with (CAPTURE / 'capacity160_on_receiver/receipts.jsonl').open() as f:
        for line in f:
            r = json.loads(line)
            if r['kind'] != 'B':
                continue
            b = by_line.pop(r['event'] + 1)
            require(r['receipt_ros_ns'] == b.arrival and r['metadata']['source'] == b.source,
                    'receipt/journal mismatch')
            latest_receipt[(b.source, b.i, b.j)] = r['metadata']['source_revision']
    require(not by_line, 'belief missing receipt metadata')
    output = {}
    for source, path in SOURCE_LOGS.items():
        latest, retired = {}, {}
        status = Counter()
        markers = ('GLIM_CBS_EXPORT_BELIEF_ROW,', 'GLIM_CBS_REVISION_BELIEF_ROW,', 'GLIM_CBS_REVISION_RETIRED_ROW,') \
            if source == 'glim_hesai' else ('KIMERA_CBS_REVISION_EDGE_ROW,', 'KIMERA_CBS_REVISION_EDGE_RETIRE_ROW,')
        with path.open() as f:
            for line_number, line in enumerate(f, 1):
                marker = next((m for m in markers if m in line), None)
                if marker is None:
                    continue
                fields = re.split('[\x1b\r\n]', line.split(marker, 1)[1])[0].split(',')
                retiring = 'RETIRE' in marker
                if source == 'glim_hesai' or retiring:
                    edge, revision = (int(fields[1]), int(fields[2])), int(fields[3])
                else:
                    edge, revision = (int(fields[2]), int(fields[3])), int(fields[6])
                    status[fields[7]] += 1
                if retiring:
                    require(edge in latest and latest[edge] == revision, 'retirement/source revision mismatch')
                    require(edge not in retired, 'duplicate source retirement marker')
                    retired[edge] = dict(last_source_revision=revision, log_line=line_number)
                else:
                    require(edge not in retired, 'source revision after source retirement')
                    latest[edge] = revision
        require(all(latest_receipt.get((source, *edge)) == rev for edge, rev in latest.items()),
                'latest source revision missing from receipt journal')
        admitted = {(b.i, b.j) for b in model.ledger.values() if b.source == source}
        output[source] = dict(raw_ids=len(latest), raw_retirement_markers=len(retired),
                              admitted_ids=len(admitted), admitted_source_retired=len(admitted & retired.keys()),
                              admitted_not_source_retired=len(admitted - retired.keys()),
                              all_last_source_versions_received=True, version_statuses=dict(status),
                              terminal_markers_forwarded_by_current_transport=False,
                              retired=[dict(from_index=e[0], to_index=e[1], **r) for e, r in retired.items()])
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    require(not args.out.exists(), 'refuse output overwrite')
    protected = [JOURNAL, *sorted(ORACLE.glob('*')), CODE / 'DiagnosticReplay.cpp', CODE / 'SourceWnoaModel.h',
                 CODE / 'driver.py', REPO / 'online_fgo/include/offline_process/CausalSupportClock.h',
                 REPO / 'online_fgo/src/solver/IncrementalFixedLagSmoother.cpp',
                 REPO / 'online_fgo/include/solver/IncrementalFixedLagSmoother.h',
                 ORACLE.parent / 'build/replay', *SOURCE_LOGS.values(),
                 CAPTURE / 'capacity160_on_receiver/receipts.jsonl',
                 CAPTURE / 'capacity160_on_receiver/resolved_config.json',
                 CAPTURE / 'capacity160_on_receiver/transport_complete.json',
                 CAPTURE / 'capacity160_on_receiver/transport_code/live_receiver_protocol.py',
                 CAPTURE / 'capacity160_on_receiver/transport_code/grandtour_live_receiver_ros1.py']
    before = {str(p): digest(p) for p in protected if p.is_file()}
    with JOURNAL.open(newline='') as f:
        model = JournalModel().read(iter(csv.reader(f)))
    reconcile(rows(ORACLE / 'support_clock.csv'), model.clock, 'support clock')
    reconcile(rows(ORACLE / 'causal_dispatch.csv'), model.dispatch, 'dispatch')
    reconcile(rows(ORACLE / 'unsupported_beliefs.csv'), model.excluded, 'unsupported')
    reconcile(rows(ORACLE / 'stream_progress.csv'), model.progress, 'event progress excluding wall time')
    old = json.loads((ORACLE / 'summary.json').read_text())
    require(len(model.clock) == old['support_states'] and len(model.ledger) == old['beliefs_accepted'], 'summary counts')
    require(model.queued == old['queued_arrival_events'] and model.pending_at_end == old['pending_at_end'], 'pending counts')
    outcome = Counter(v['outcome'] for v in model.dispatch)
    require(outcome == dict(insert=2201, replace=86053), 'unexpected frozen outcome counts')
    result, versions, identities, boundaries, absorbed = analyze(model)
    terminal = terminal_audit(model)
    result['source_retirement_evidence'] = {source: {k: v for k, v in info.items() if k != 'retired'}
                                          for source, info in terminal.items()}
    after = {p: digest(Path(p)) for p in before}
    require(before == after, 'protected input changed')
    result.update(validation='PASS', estimator_executed=False, event_counts=dict(model.event_counts),
                  supports=len(model.clock), belief_versions=len(model.beliefs),
                  exclusions=dict(Counter(v['reason'] for v in model.excluded)),
                  queued_versions=model.queued, peak_pending=model.max_pending,
                  origin_ns=model.origin, end_marker_ns=model.end, final_support_ns=model.clock[-1]['physical_timestamp_ns'],
                  final_imu_ns=model.previous_physical,
                  exact_oracle_matches=dict(support_rows=len(model.clock), dispatch_rows=len(model.dispatch),
                                            exclusion_rows=len(model.excluded), progress_rows=len(model.progress)),
                  protected_hashes_unchanged=True, protected_files=len(before),
                  note='Metadata feasibility only; last observed is not finalized. No optimizer/covariance/timing result.')
    args.out.mkdir(parents=True)
    save_json(args.out / 'protected_hashes.json', before)
    save_csv(args.out / 'dispatched_support_ages.csv', versions)
    save_csv(args.out / 'stable_ids.csv', identities)
    save_csv(args.out / 'lifecycle.csv', boundaries)
    save_csv(args.out / 'absorbed_ids.csv', absorbed)
    save_json(args.out / 'source_terminal_markers.json', terminal)
    save_json(args.out / 'analysis_source_hashes.json', {str(p): digest(p) for p in Path(__file__).parent.iterdir() if p.is_file()})
    save_json(args.out / 'summary.json', result)
    save_json(args.out / 'complete.json', dict(passed=True, estimator_executed=False))
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == '__main__':
    main()
