"""Three bounded matched saved-journal repeats; refuse overwrites/retries."""
from pathlib import Path
import hashlib
import json
import os
import shlex
import subprocess
import threading
import itertools
from experiment import ROOT, BINARY, ENV, file_sha
from numerical_equivalence import compare, check_bounds, LIMITS

out = ROOT/'repeatability'
out.mkdir(exist_ok=False)
stream = ROOT/'gate_on_receiver/events.csv'
live = ROOT/'gate_on_receiver/receiver'
assert file_sha(BINARY) == 'fb56ffc5c6c9cb575d6129289688caeabb4e3baba32e429b857f3d6a30e50d1f'
for key in LIMITS:
    values = dict.fromkeys(LIMITS, 0.)
    for bad in [LIMITS[key]*1.01, float('nan')]:
        values[key] = bad
        try: check_bounds(values)
        except AssertionError: pass
        else: raise AssertionError('Comparator must reject '+key)
manifest = {'input_sha256': file_sha(stream), 'binary_sha256': file_sha(BINARY),
            'limits': LIMITS, 'environment': ENV,
            'validator_sha256': file_sha(ROOT/'numerical_equivalence.py'),
            'protocol_sha256': file_sha(ROOT/'repeatability_protocol.txt')}
setup = 'source /opt/ros/humble/setup.bash && '
diag = 'uname -a; ' + shlex.join(['env', *[k+'='+v for k,v in sorted(ENV.items())], 'ldd', str(BINARY)])
r = subprocess.run(['docker','exec','ubuntu-jammy-humble-nvidia','bash','-lc',setup+diag], capture_output=True, text=True, check=True)
(out/'runtime.txt').write_text(r.stdout+r.stderr)
manifest['runtime_sha256'] = file_sha(out/'runtime.txt')
(out/'manifest.json').write_text(json.dumps(manifest,indent=2))
directories = []
for label in ['file_a','file_b','fifo_c']:
    dest = out/label
    source = stream
    errors = []
    writer = None
    if label == 'fifo_c':
        source = out/'journal.fifo'
        os.mkfifo(source, 0o600)
        def feed():
            try:
                with source.open('w') as sink, stream.open() as src:
                    for line in src: sink.write(line); sink.flush()
            except BaseException as e: errors.append(repr(e))
        writer = threading.Thread(target=feed, daemon=True)
        writer.start()
    command = ['env', *[k+'='+v for k,v in sorted(ENV.items())], str(BINARY),
               '--event-stream',str(source),'--out',str(dest),'--mode','sources',
               '--timeline','record','--solver','isam2','--support-policy','uniform_imu',
               '--support-period-ns','100000000','--receiver-startup-samples','59',
               '--receiver-model','source_wnoa']
    cmd = ['docker','exec','ubuntu-jammy-humble-nvidia','bash','-lc',setup+'exec '+shlex.join(command)]
    (out/(label+'_command.json')).write_text(json.dumps(cmd,indent=2))
    r = subprocess.run(cmd,capture_output=True,text=True,timeout=60)
    (out/(label+'.log')).write_text(r.stdout+r.stderr)
    assert r.returncode == 0, (label,r.stderr)
    if writer: writer.join(5); assert not writer.is_alive() and not errors
    result = compare(live,dest)
    (out/(label+'_comparison.json')).write_text(json.dumps(result,indent=2))
    directories.append(dest)
    print(label+': numerical equivalence PASSED',flush=True)
pairwise = {}
for a,b in itertools.combinations(directories,2): pairwise[a.name+'__'+b.name] = compare(a,b)
assert file_sha(stream) == manifest['input_sha256']
result = {'passed': True, 'three_repeats_passed': True, 'pairwise': pairwise,
          'comparison_to_live': {p.name:json.loads((out/(p.name+'_comparison.json')).read_text()) for p in directories},
          'limitation': 'Bounded repeatability under frozen settings, not proof of the exact floating-point cause.'}
(out/'validation.json').write_text(json.dumps(result,indent=2))
print('ALL THREE REPEATS AND PAIRWISE COMPARISONS PASSED')
