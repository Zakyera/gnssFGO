#!/usr/bin/env python3
"""Freeze launch inputs only; does not run an estimator."""
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent
OLD = ROOT.parent/'gnssfgo_eth1_live120_zero_20260930'
BINARY = Path('/home/yeranis/results/gnssfgo_eth1_local_prediction_20261007/build/wnoa/replay')
RUNTIME = Path('/home/yeranis/results/gnssfgo_synthetic_uncertainty_20261004/baseline/runtime/lib')
REPO = Path('/home/yeranis/repos/gnssFGO')
ENV = dict(LD_LIBRARY_PATH=str(RUNTIME), OMP_NUM_THREADS='1',
           WNOJ_STRICT_PUBLICATION='1', WNOJ_INCREMENTAL_PROFILE='nominal')

def exclusive(path, text):
    with path.open('x') as f:
        f.write(text)

def main():
    env = (OLD/'source.env').read_text().replace(OLD.name, ROOT.name)
    env = env.replace('export CBSMS_BAG_DURATION_SEC=120.0',
                      ': "${CBSMS_BAG_DURATION_SEC:?Set explicit diagnostic/full duration}"\nexport CBSMS_BAG_DURATION_SEC')
    env = env.replace('11368', '11378')
    exclusive(ROOT/'source.env', env)
    for phase, seconds in [('smoke', 30), ('full', 346)]:
        for mode in ['off', 'on']:
            config = json.loads((OLD/f'receiver_{mode}.json').read_text())
            config.update(sequence=f'ETH1 2024-10-01-11-29-55; offset 0; {seconds}s; classic CBS {mode.upper()}; WNOA no central IMU',
                          receiver_model='source_wnoa', receiver_binary=str(BINARY),
                          receiver_environment=ENV, ros_master_uri='http://127.0.0.1:11378')
            for source in config['sources']:
                source['session'] = f'eth1_wnoa_{phase}_{mode}_20261007'
            exclusive(ROOT/f'{phase}_{mode}.json', json.dumps(config, indent=2)+'\n')
    files = [BINARY, *RUNTIME.glob('*'), REPO/'tools/live_receiver_protocol.py',
             REPO/'tools/grandtour_live_receiver_ros1.py', REPO/'tools/launch_grandtour_live_receiver.py']
    profile = ROOT.parent/'grandtour_eth1_presentation/grandtour_eth1_stereo_shared_cpt7_no_sidecar_cbs_off_full_346s_20260907'
    for directory in ['glim_config', 'kimera_profile']:
        files += sorted(p for p in (profile/directory).rglob('*') if p.is_file())
    exclusive(ROOT/'protected_manifest.json', json.dumps({str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in files},indent=2)+'\n')

if __name__ == '__main__':
    main()
