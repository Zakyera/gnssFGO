"""Saved-only analysis build correction; never recaptures any sensor data."""
from pathlib import Path
H=Path(__file__).resolve().parent
source=(H/'run_saved.py').read_text()
# Preserve first compile failure and pinned source. GTSAM StereoCamera fixture
# constructor takes a shared calibration pointer, not a calibration value.
source=source.replace("D=K/'analysis'", "D=K/'analysis_v2'")
source=source.replace('VisualStudy.cpp','VisualStudy_v2.cpp').replace("/analysis/","/analysis_v2/")
source=source.replace("H/'run_saved.py'", "H/'run_saved_v2.py'")
exec(compile(source,str(H/'run_saved.py')+' (offline v2)','exec'))
