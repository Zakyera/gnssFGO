from pathlib import Path
import difflib
root=Path(__file__).resolve().parent
repo=Path('/home/yeranis/repos/gnssFGO')
old=root.parent/'gnssfgo_eth1_bounded_polish_20261007'
for model in ['wnoa','wnoj']:
    d=root/'build'/model
    assert not (d/'replay').exists()
    after=(repo/'online_fgo/include/offline_process/FullHistoryPublicationRefinement.h').read_text()
    before=(old/'build'/model/'FullPublicationRefinement.h').read_text()
    (d/'FullPublicationRefinement.h').write_text(after)
    (root/f'{model}_local_prediction.diff').write_text(''.join(difflib.unified_diff(before.splitlines(True),after.splitlines(True))))
    p=d/'DiagnosticReplay.cpp'
    text=p.read_text()
    text=text.replace('recovery,consensus,qr_disagreement\\n', 'recovery,consensus,qr_disagreement,local_prediction\\n')
    text=text.replace("<<s.qr_disagreement<<'\\n';", "<<s.qr_disagreement<<','<<s.local_prediction<<'\\n';")
    p.write_text(text)
