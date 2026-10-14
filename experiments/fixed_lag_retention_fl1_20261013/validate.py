"""Retain FL1 unit/integrity/deterministic metadata checks, without estimators."""
import io
import json
from pathlib import Path
import unittest
from analyze import digest, require, rows, save_json
import test_metadata


def main():
    root = Path('/home/yeranis/results/gnssfgo_fixed_lag_retention_fl1_20261013')
    log = io.StringIO()
    result = unittest.TextTestRunner(stream=log, verbosity=2).run(
        unittest.defaultTestLoader.loadTestsFromModule(test_metadata))
    require(result.wasSuccessful(), log.getvalue())
    first, second = root / 'analysis3', root / 'analysis4'
    filenames = ['summary.json', 'protected_hashes.json', 'dispatched_support_ages.csv',
                 'stable_ids.csv', 'lifecycle.csv', 'absorbed_ids.csv',
                 'source_terminal_markers.json', 'complete.json']
    comparisons = {name: digest(first / name) == digest(second / name) for name in filenames}
    require(all(comparisons.values()), 'nondeterministic metadata products')
    manifest = json.loads((first / 'protected_hashes.json').read_text())
    unchanged = {p: digest(Path(p)) == h for p, h in manifest.items()}
    require(all(unchanged.values()), 'protected source modified')
    markers = json.loads((first / 'source_terminal_markers.json').read_text())
    terminal_ids = {(source, r['from_index'], r['to_index'])
                    for source, entry in markers.items() for r in entry['retired']}
    absorbed = rows(first / 'absorbed_ids.csv')
    missing = [r for r in absorbed if (r['source'], int(r['from_index']), int(r['to_index'])) not in terminal_ids]
    require(not missing, 'absorbed ID lacks retained source-retirement evidence')
    save_json(root / 'validation.json', dict(passed=True, estimator_executed=False,
                unit_tests=result.testsRun, unit_log=log.getvalue(), deterministic_products=comparisons,
                protected_files_unchanged=unchanged, absorbed_ids=len(absorbed),
                absorbed_without_source_retirement_marker=len(missing),
                incomplete_attempt='analysis1 had CSV schema output error, not an estimator failure; retained, no complete.json',
                superseded_attempt='analysis2 passed core bookkeeping, superseded by analysis3/4 source-terminal audit'))
    print(f'PASS: {result.testsRun} unit tests; {len(comparisons)} identical outputs; '
          f'{len(unchanged)} frozen files unchanged; {len(absorbed)} absorbed IDs source-retired.')


if __name__ == '__main__':
    main()
