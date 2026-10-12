"""Print only profiler configuration emitted by its official launch frontend."""
import os
import json
print(json.dumps({k:v for k,v in os.environ.items() if k.startswith(('SP_COLLECTOR','GPROFNG')) or k=='LD_PRELOAD'},indent=2))
