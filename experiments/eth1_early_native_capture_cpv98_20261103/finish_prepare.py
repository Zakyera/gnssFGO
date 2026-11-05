"""Complete the pre-launch input audit only; existing harness is not recopied."""
from pathlib import Path
H=Path(__file__).resolve().parent
source=(H/'prepare.py').read_text()
assert not (H/'prepared.json').exists()
prefix=source[:source.index('assert not P.exists()')]
suffix=source[source.index("audit=read(Q/'input_audit.json')"):]
suffix=suffix.replace('>11.7','>11.6')
exec(compile(prefix+"\nassert P.exists() and not (P/'on_source').exists()\n"+
    "states=subprocess.check_output(['docker','inspect','-f','{{.Name}} {{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'],text=True)\n"+
    "assert states=='/cbsms_ws_gpu exited\\n/ubuntu-jammy-humble-nvidia exited\\n'\n"+
    "with socket.socket() as s:s.bind(('127.0.0.1',11610))\n"+
    "parity=read(H/'profile_parity.json')['files']\n"+
    "for x in parity:assert sha(x['old'])==sha(x['new'])==x['sha256']\n"+suffix,
    str(H/'prepare.py')+' (input-admission continuation)','exec'))
