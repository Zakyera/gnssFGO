"""Mechanical private copy/rebase only; does not launch, build or alter inputs."""
import shutil
from pathlib import Path
H = Path(__file__).resolve().parent
W = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
S = W/'runs/eig2_hesai_insertion_cpv51_20261030'
K = W/'runs'/H.name
assert not K.exists()
K.mkdir(); (K/'private_build').mkdir()
shutil.copytree(S/'include', K/'include')
for p in S.iterdir():
    if p.suffix in ('.h', '.inc', '.cpp') or p.name == 'build.py':
        shutil.copy2(p, K/p.name)
for p in K.rglob('*'):
    if p.is_file():
        old = p.read_text()
        new = old.replace(S.name, K.name).replace('CPV51_AUDIT_DIR', 'CPV66_AUDIT_DIR').replace('CPV51_TEST_ALL', 'CPV66_TEST_ALL')
        if new != old: p.write_text(new)
S2 = H.parent/S.name
for name in ('build_controller.py','prepare.py','controller.py','launch.py','validate_snapshots.py'):
    old = (S2/name).read_text()
    new = old.replace('CPV51', 'CPV66').replace('cpv51', 'cpv66').replace('CP-V51', 'CP-V66').replace('11551', '11566')
    with (H/name).open('x') as f: f.write(new)
print(K)
