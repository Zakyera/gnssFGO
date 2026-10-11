"""Prepare immutable, GT-free mean adapter inputs from retained capture."""
import csv
import hashlib
import json
import struct
from pathlib import Path

ROOT = Path(__file__).resolve().parent
RUNS = ROOT.parent
OFF = RUNS / 'gnssfgo_eth1_active_window40_off_20261008'
ON = RUNS / 'gnssfgo_eth1_active_window40_prepared_20261008'
AUDIT = OFF / 'insertion_retirement_audit_20261008'


def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1048576), b''):
            h.update(block)
    return h.hexdigest()


def write_json(path, data):
    with path.open('x') as stream:
        json.dump(data, stream, indent=2, allow_nan=False)


def main():
    assert not (ROOT / 'manifest.json').exists()
    source = json.loads((AUDIT / 'wire_beliefs.json').read_text())
    beliefs = {(v['i'], v['j']): v for v in source['beliefs']}
    outputs = {v['update']: v for v in source['kimera_outputs']}
    with (AUDIT / 'accepted_insertions.csv').open() as stream:
        accepted = list(csv.DictReader(stream))
    assert len(accepted) == 175
    events_path = ON / 'pilot_on_receiver/events.csv'
    receipts_path = ON / 'pilot_on_receiver/receipts.jsonl'
    imu = []
    with events_path.open() as events, receipts_path.open() as receipts:
        for index, (line, receipt_line) in enumerate(zip(events, receipts, strict=True)):
            row = line.strip().split(',')
            receipt = json.loads(receipt_line)
            assert index == receipt['event'] and row[0] == receipt['kind']
            if row[0] != 'I':
                continue
            arrival, stamp = map(int, row[1:3])
            assert arrival == receipt['receipt_ros_ns']
            assert len(row) == 9
            assert not imu or stamp > imu[-1][0]
            imu.append((stamp, arrival, *map(float, row[3:])))
    with (ROOT / 'imu.txt').open('x') as stream:
        for row in imu:
            stream.write(' '.join(str(x) if isinstance(x, int) else format(x, '.17g') for x in row)+'\n')
    metadata = []
    with (ROOT / 'queries.txt').open('x') as stream:
        for ordinal, event in enumerate(accepted):
            u, i, j = map(int, (event['update'], event['i'], event['j']))
            belief = beliefs[(int(event['sender_i']), int(event['sender_j']))]
            c, d = outputs[i]['header_ns'], outputs[j]['header_ns']
            a, b = round(belief['ti']*1e9), round(belief['tj']*1e9)
            arrival = belief['record_ns']
            assert max(a, b, c, d) <= arrival
            # The original wire uses floating-point seconds for source stamps;
            # receiver stamps retain exact ROS nanoseconds. Do not pretend that
            # a double epoch recovers sub-double source timestamp precision.
            assert abs(c*1e-9 - (float(event['receiver_start_s'])+1727775008.003475)) < 1e-6
            stream.write(' '.join(map(str, (ordinal, c, a, b, d, arrival)))+' '+
                         ' '.join(format(x,'.17g') for x in belief['mu'])+'\n')
            metadata.append(dict(ordinal=ordinal, update=u, receiver_i=i, receiver_j=j,
                                 receiver_start_ns=c, receiver_end_ns=d, source_start_ns=a,
                                 source_end_ns=b, arrival_ns=arrival, original=belief,
                                 covariance_sha256=hashlib.sha256(struct.pack('<36d',*belief['covariance'])).hexdigest()))
    write_json(ROOT/'metadata.json', metadata)
    protected = [AUDIT/'wire_beliefs.json', AUDIT/'accepted_insertions.csv',
                 AUDIT/'reference.npz', events_path, receipts_path,
                 ON/'pilot_on_source/effective_mode_check.json',
                 ON/'pilot_on_source/estimators.log', OFF/'pilot_off_source/estimators.log']
    header = Path('/home/yeranis/repos/ethzasl_msf/msf_core/include/msf_core')
    protected.extend(header/name for name in ['msf_cbs_imu_bridge_history.h',
                                             'msf_cbs_imu_dynamics.h','msf_se3.h'])
    write_json(ROOT/'manifest.json', dict(
        inputs={str(p):sha(p) for p in protected}, imu_samples=len(imu), queries=len(metadata),
        initial_timestamp_ns=1727775008675043000, reference_excluded_from_adapter=True,
        covariance_policy='Verbatim packed-double copy. No covariance transformation or scaling.'))
    print(json.dumps(dict(imu_samples=len(imu), queries=len(metadata))))


if __name__ == '__main__':
    main()
