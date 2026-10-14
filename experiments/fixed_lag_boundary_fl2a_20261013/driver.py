"""Reuse the frozen create-once build harness; synthetic executables only."""
import importlib.util
from pathlib import Path
HERE=Path(__file__).resolve().parent
OLD=HERE.parent/'fixed_lag_receiver_fl2_20261013'
spec=importlib.util.spec_from_file_location('fl2driver',OLD/'driver.py')
base=importlib.util.module_from_spec(spec);spec.loader.exec_module(base)
original_protected=base.protected
def protected():
    d=original_protected()
    for p in [*OLD.glob('*'),
      Path('/home/yeranis/results/gnssfgo_fixed_lag_receiver_fl2_20261013/build_release5'),
      Path('/home/yeranis/results/gnssfgo_fixed_lag_receiver_fl2_20261013/validation.json')]:
        if p.is_file():d[str(p)]=base.digest(p)
    return d
base.protected=protected
base.HERE=HERE
base.ROOT=Path('/home/yeranis/results/gnssfgo_fixed_lag_boundary_fl2a_20261013')
if __name__=='__main__':base.main()
