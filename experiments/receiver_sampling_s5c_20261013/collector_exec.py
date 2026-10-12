"""Set collector founder to the exec target PID, as the frontend does."""
import os
import sys
env=os.environ.copy()
env.update(__import__('json').loads(sys.argv[1]))
env['SP_COLLECTOR_FOUNDER']=str(os.getpid())
os.execvpe(sys.argv[2],sys.argv[2:],env)
