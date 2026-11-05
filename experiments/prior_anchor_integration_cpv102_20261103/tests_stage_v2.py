"""Same bounded tests, fresh receipts after predecessor-exit prelaunch rejection."""
from pathlib import Path
import sys

assert sys.argv[1:]==[]
runner=Path(__file__).with_name('build_stage.py')
source=runner.read_text()
old='D=H/stage;D.mkdir()'
assert source.count(old)==1
source=source.replace(old,"D=H/'tests_v2';D.mkdir()")
sys.argv=[str(runner),'tests']
exec(compile(source,str(runner),'exec'),{'__file__':str(runner),'__name__':'__main__'})
