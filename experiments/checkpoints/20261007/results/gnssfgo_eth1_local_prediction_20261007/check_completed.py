from pathlib import Path
import subprocess,sys
root=Path(__file__).resolve().parent
for label in sys.argv[1:]:
 subprocess.run([sys.executable,str(root/'validate.py'),label],check=True)
