from pathlib import Path
import shutil,difflib

root=Path(__file__).resolve().parent
old=root.parent/'gnssfgo_eth1_bounded_polish_20261007'
repo=Path('/home/yeranis/repos/gnssFGO')
assert not (root/'manifest.json').exists()
(root/'logs').mkdir()
for model in ['wnoa','wnoj']:
    dest=root/'build'/model
    shutil.copytree(old/'build'/model,dest,ignore=shutil.ignore_patterns('replay','diagnostic'))
    p=dest/'FullPublicationRefinement.h'
    before=p.read_text()
    after=(repo/'online_fgo/include/offline_process/FullHistoryPublicationRefinement.h').read_text()
    p.write_text(after)
    (root/f'{model}_local_prediction.diff').write_text(''.join(difflib.unified_diff(before.splitlines(True),after.splitlines(True))))
    p=dest/'DiagnosticReplay.cpp'
    text=p.read_text()
    text=text.replace('recovery,consensus,qr_disagreement\\n', 'recovery,consensus,qr_disagreement,local_prediction\\n')
    text=text.replace("<<s.qr_disagreement<<'\\n';", "<<s.qr_disagreement<<','<<s.local_prediction<<'\\n';")
    p.write_text(text)
for name in ['run.py','run.sh','validate.py','summarize.py','check_completed.py','final_statistics.py','archive_sources.py']:
    (root/name).write_text((old/name).read_text().replace(str(old),str(root)))
(root/'manifest.json').write_text((old/'manifest.json').read_text())
