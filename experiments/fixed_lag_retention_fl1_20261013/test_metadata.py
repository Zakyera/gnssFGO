import unittest
import tempfile
from pathlib import Path
from analyze import JournalModel, NS, binding, choose_lag, simulate, reconcile, save_csv, rows

BASE = 100 * NS
ORIGIN = BASE + NS


def imu(t, arrival=None):
    return ['I', str(t if arrival is None else arrival), str(t)] + ['0'] * 6


def belief(ti, tj, arrival, revision=0, source='lidar', session='one', i=1, j=2, payload='0'):
    return ['B', str(arrival), source, session, str(i), str(revision), str(i), str(j), str(ti), str(tj)] + [payload] * 42


def journal(tail):
    records = [imu(BASE), imu(ORIGIN)] + tail
    return JournalModel(startup=2, period=NS).read(iter(records + [['E', records[-1][1]]]))


class MetadataTests(unittest.TestCase):
    def test_output_schema_when_source_first_appears(self):
        with tempfile.TemporaryDirectory(prefix='fl1_metadata_') as folder:
            path = Path(folder) / 'result.csv'
            save_csv(path, [dict(support=0), dict(support=1, source_a=2)])
            self.assertEqual(rows(path)[1], dict(support='1', source_a='2'))
            with self.assertRaises(FileExistsError):
                save_csv(path, [dict(support=2)])

    def test_integer_brackets_and_before_origin(self):
        self.assertEqual(binding(ORIGIN, ORIGIN, NS), (0,))
        self.assertEqual(binding(ORIGIN + 1, ORIGIN, NS), (0, 1))
        self.assertEqual(binding(ORIGIN + NS - 1, ORIGIN, NS), (0, 1))
        with self.assertRaisesRegex(ValueError, 'before_initialization'):
            binding(ORIGIN - 1, ORIGIN, NS)

    def test_lag_margin_rounding(self):
        self.assertEqual(choose_lag(5 * NS), 6 * NS)
        self.assertEqual(choose_lag(5 * NS + 1), 7 * NS)

    def test_pending_and_clock_delay(self):
        m = journal([imu(ORIGIN + NS),
                     belief(ORIGIN + NS // 5, ORIGIN + 6 * NS // 5, ORIGIN + 13 * NS // 10),
                     imu(ORIGIN + 2 * NS, ORIGIN + 4 * NS)])
        self.assertEqual(m.queued, 1)
        self.assertEqual(m.dispatch[0]['insertion_ns'], ORIGIN + 4 * NS)
        self.assertEqual(m.dispatch[0]['created_support'], 2)
        result, _, _ = simulate(m, 2 * NS)
        self.assertEqual(result['active_ids_at_end'], 1)  # Wall time does not retire 0.

    def test_exact_boundary_retained_then_absorbed(self):
        m = journal([imu(ORIGIN + NS), belief(ORIGIN, ORIGIN + NS, ORIGIN + NS),
                     imu(ORIGIN + 2 * NS), imu(ORIGIN + 3 * NS)])
        result, history, _ = simulate(m, 2 * NS)
        self.assertEqual(history[2]['absorbed_ids'], 0)
        self.assertEqual(history[3]['absorbed_ids'], 1)
        self.assertEqual(result['remaining_supports'], 3)
        self.assertEqual(result['peak_before_retirement']['supports'], 4)

    def test_revision_before_retirement(self):
        m = journal([imu(ORIGIN + NS), belief(ORIGIN, ORIGIN + NS, ORIGIN + NS),
                     imu(ORIGIN + 2 * NS), belief(ORIGIN, ORIGIN + NS, ORIGIN + 29 * NS // 10, 1),
                     imu(ORIGIN + 3 * NS)])
        result, _, absorbed = simulate(m, 2 * NS)
        self.assertEqual(result['dispatch_counts']['replace'], 1)
        self.assertEqual(absorbed[0]['last_revision'], 1)

    def test_revision_after_retirement_fails(self):
        m = journal([imu(ORIGIN + NS), belief(ORIGIN, ORIGIN + NS, ORIGIN + NS),
                     imu(ORIGIN + 3 * NS), belief(ORIGIN, ORIGIN + NS, ORIGIN + 3 * NS, 1)])
        with self.assertRaisesRegex(ValueError, 'after absorption'):
            simulate(m, 2 * NS)

    def test_pending_old_bracket_fails_before_retirement(self):
        m = journal([imu(ORIGIN + NS),
                     belief(ORIGIN, ORIGIN + 42 * NS // 10, ORIGIN + 42 * NS // 10),
                     imu(ORIGIN + 3 * NS, ORIGIN + 5 * NS)])
        with self.assertRaisesRegex(ValueError, 'pending old bracket'):
            simulate(m, 2 * NS)

    def test_source_session_and_version_identity(self):
        tail = [imu(ORIGIN + NS)]
        for source, session in [('a', 's'), ('b', 's'), ('a', 'new')]:
            tail.append(belief(ORIGIN, ORIGIN + NS, ORIGIN + NS, source=source, session=session))
        tail += [belief(ORIGIN, ORIGIN + NS, ORIGIN + NS, 1, 'a', 's'),
                 belief(ORIGIN, ORIGIN + NS, ORIGIN + NS, 1, 'a', 's'),
                 belief(ORIGIN, ORIGIN + NS, ORIGIN + NS, 0, 'a', 's')]
        result, _, _ = simulate(journal(tail), 2 * NS)
        self.assertEqual(result['dispatch_counts'], dict(insert=3, replace=1, duplicate=1, stale=1))
        self.assertEqual(result['active_ids_at_end'], 3)

    def test_conflicting_duplicate_fails(self):
        with self.assertRaisesRegex(ValueError, 'nonidentical duplicate'):
            journal([imu(ORIGIN + NS), belief(ORIGIN, ORIGIN + NS, ORIGIN + NS),
                     belief(ORIGIN, ORIGIN + NS, ORIGIN + NS, payload='1')])

    def test_original_exclusions_and_eof(self):
        m = journal([belief(BASE, ORIGIN, ORIGIN),
                     belief(ORIGIN, ORIGIN + NS // 2, ORIGIN + NS // 2)])
        self.assertEqual([r['reason'] for r in m.excluded], ['before_initialization', 'missing_right_support_at_end'])
        self.assertEqual(m.pending_at_end, 1)

    def test_identical_arrival_raw_order(self):
        m = journal([belief(ORIGIN, ORIGIN + NS, ORIGIN + NS),
                     imu(ORIGIN + NS), belief(ORIGIN, ORIGIN + NS, ORIGIN + NS, 1)])
        self.assertEqual([d['outcome'] for d in m.dispatch], ['insert', 'replace'])
        self.assertEqual(m.queued, 1)

    def test_unmarked_eof_fails(self):
        with self.assertRaisesRegex(ValueError, 'missing E'):
            JournalModel(startup=2).read(iter([imu(BASE), imu(ORIGIN)]))

    def test_oracle_mismatch_fails(self):
        with self.assertRaises(ValueError):
            reconcile([dict(index=2)], [dict(index=1)], 'fixture')

    def test_deterministic_repeat(self):
        tail = [imu(ORIGIN + NS), belief(ORIGIN, ORIGIN + NS, ORIGIN + NS), imu(ORIGIN + 5 * NS)]
        self.assertEqual(simulate(journal(tail), 2 * NS), simulate(journal(tail), 2 * NS))


if __name__ == '__main__':
    unittest.main(verbosity=2)
