"""Independent checks of saved audit products; no estimator execution."""
import json
from pathlib import Path

import numpy as np

import analyze as a


def main():
    out = a.HERE/'verification.json'
    assert not out.exists()
    result = {}
    assert a.self_test()
    for name in ['eig2', 'spx3']:
        path = a.HERE/(name+'_results.json')
        data = json.loads(path.read_text())
        for p, expected in data['input_hashes'].items():
            assert a.sha(Path(p)) == expected, p
        root = a.WORK/'runs'/a.CASES[name][0]/'prepared'
        off = a.snapshots(root/'live60_off_source/estimators.log')
        off_post = []
        maximum_metric_difference = 0.
        maximum_rotation_orthogonality = 0.
        translation_wins = 0
        for r in data['records']:
            gt = np.asarray(r['receiver_gt'])
            for key, T in r['transforms'].items():
                T = np.asarray(T)
                reference = np.asarray(r['sender_gt']) if key == 'raw' else gt
                R = T[:3, :3]
                maximum_rotation_orthogonality = max(maximum_rotation_orthogonality,
                    float(np.max(np.abs(R.T@R-np.eye(3)))))
                translation = np.linalg.norm(T[:3, 3]-reference[:3, 3])
                # Independent trace/atan2 formula, not Rotation.as_rotvec.
                D = R.T@reference[:3, :3]
                skew = np.array([D[2,1]-D[1,2],D[0,2]-D[2,0],D[1,0]-D[0,1]])
                angle = np.arctan2(np.linalg.norm(skew)/2,(np.trace(D)-1)/2)*180/np.pi
                maximum_metric_difference = max(maximum_metric_difference,
                   abs(translation-r['errors'][key]['translation_m']),
                   abs(angle-r['errors'][key]['rotation_deg']))
            cur, i, j = r['update'],r['receiver_from'],r['receiver_to']
            T = a.inv(off[cur,i])@off[cur,j]
            off_post.append(a.error(T,gt))
            translation_wins += r['errors']['aligned']['translation_m'] < r['errors']['off_pre']['translation_m']
        assert maximum_metric_difference < 1e-10
        assert maximum_rotation_orthogonality < 1e-12
        result[name] = dict(input_hashes_unchanged=True,rows=len(data['records']),
            max_independent_metric_difference=maximum_metric_difference,
            max_rotation_orthogonality_error=maximum_rotation_orthogonality,
            aligned_translation_better_than_off_pre_count=int(translation_wins),
            off_same_update_postsolve={k:a.stats([e[k] for e in off_post]) for k in off_post[0]},
            audit_product_sha256=a.sha(path))
    with out.open('x') as f:
        json.dump(result,f,indent=2,allow_nan=False)
    print(json.dumps(result,indent=2))


if __name__ == '__main__':
    main()
