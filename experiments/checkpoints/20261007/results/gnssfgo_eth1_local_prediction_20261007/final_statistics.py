from pathlib import Path
import csv, json, hashlib

root = Path(__file__).resolve().parent
out = {}
for mode in ['off', 'on']:
    for model in ['wnoa', 'wnoj']:
        name = f'{mode}_{model}'
        case = root / name
        with (case / 'publication_convergence.csv').open() as f:
            conv = list(csv.DictReader(f))
        with (case / 'publication_steps.csv').open() as f:
            steps = list(csv.DictReader(f))
        recoveries = [r for r in steps if r['recovery'] == '1']
        accepted = [r for r in recoveries if r['accepted'] == '1']
        count = {}
        for row in recoveries:
            key = (row['phase'], row['index'])
            count[key] = count.get(key, 0) + 1
        out[name] = dict(
            complete=(case / 'summary.json').exists(),
            publications=len(conv),
            maximum_passes=max(int(r['attempted']) for r in conv),
            maximum_gradient=max(float(r['gradient_after']) for r in conv),
            recovery_publications=len(count),
            private_polish_publications=[dict(phase=k[0], index=int(k[1]), candidates=n)
                                        for k,n in count.items() if n > 1],
            accepted_recoveries=accepted,
            maximum_accepted_positive_relative_cost=max(
                [0.] + [float(r['stable_cost_delta']) / max(1., float(r['error_before']))
                        for r in accepted]),
        )
        for suffix in ['', '_repeat']:
            p = root / f'validation_{name}{suffix}.json'
            if p.exists():
                data = json.loads(p.read_text())
                out[name]['validation' + suffix] = {k: data.get(k) for k in
                    ['pass_all', 'repeat_bitwise', 'repeat_state_differences',
                     'repeat_numerical', 'accounting', 'numerical_prefix']}
        out[name]['trajectory_sha256'] = {
            f: hashlib.sha256((case/f).read_bytes()).hexdigest()
            for f in ['trajectory.tum', 'online_trajectory.tum'] if (case/f).exists()}
for test in ['unit', 'sanitizer']:
    out[test] = json.loads((root/f'command_{test}.json').read_text())
(root / 'final_statistics.json').write_text(json.dumps(out, indent=2) + '\n')
print(json.dumps(out, indent=2))
