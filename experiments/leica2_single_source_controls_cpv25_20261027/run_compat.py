"""Python3.10-compatible harness entry point; original failed preflight preserved."""
import hashlib, importlib.util, json, sys, traceback
from pathlib import Path
sys.dont_write_bytecode=True
H=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('control25',H/'run.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)

def sha(path):
    digest=hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda:f.read(2**20),b''):digest.update(block)
    return digest.hexdigest()

assert (H/'failed.txt').exists() and 'hashlib' in (H/'failed.txt').read_text()
assert not (H/'glim_only_launch.json').exists() and not (H/'kimera_only_launch.json').exists()
original_save=m.save
def save(path,obj):
    if path==H/'start.json':path=H/'start_compat.json'
    return original_save(path,obj)
m.sha=sha;m.save=save
original_save(H/'compatibility_pins.json',{str(p):sha(p) for p in (Path(__file__),H/'run.py',H/'input_pins.json',H/'prepared.json')})
try:
    m.main()
except Exception:
    with (H/'failed_compat.txt').open('x') as f:f.write(traceback.format_exc())
    raise
