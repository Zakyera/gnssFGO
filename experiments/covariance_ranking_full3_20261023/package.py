"""Verify the plotted-data finding and archive only this bounded analysis."""
import csv
import hashlib
import json
import math
from pathlib import Path
import zipfile

ROOT = Path(__file__).resolve().parent
OUT = ROOT / 'outputs'
ARCHIVE = ROOT / 'covariance_ranking_full3.zip'


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for b in iter(lambda: f.read(4*1024*1024), b''): h.update(b)
    return h.hexdigest()


def main():
    assert not ARCHIVE.exists() and not (ROOT/'supplement.json').exists()
    audit = json.loads((OUT/'audit.json').read_text())
    assert audit['passed'] and audit['inputs_unchanged']
    assert all(digest(Path(p)) == h for p,h in audit['input_hashes'].items())
    assert len(list(OUT.glob('*.png'))) == len(list(OUT.glob('*.svg'))) == 8
    rows = list(csv.DictReader((OUT/'samples.csv').open()))
    assert len(rows) == audit['selected_rows'] == 51068
    selection = [r for r in rows if r['sequence']=='SPX-3' and
                 r['source']=='kimera_front' and r['mode']=='on' and r['stage']=='near4s']
    sse = sum(float(r['translation_error_mm'])**2 for r in selection)
    top = sorted(selection,key=lambda r: float(r['translation_error_mm']),reverse=True)[:3]
    assert [(int(r['from_id']),int(r['to_id'])) for r in top] == [(0,1),(1,2),(2,3)]
    evidence=[]
    for r in top:
        first = next(q for q in rows if q['sequence']=='SPX-3' and
                     q['source']=='kimera_front' and q['mode']=='on' and q['stage']=='first' and
                     q['from_timestamp_ns']==r['from_timestamp_ns'] and
                     q['to_timestamp_ns']==r['to_timestamp_ns'])
        evidence.append(dict(first=first,near4s=r,
            near4s_covariance_percentile=100*sum(float(q['translation_trace_m2'])<=
                                               float(r['translation_trace_m2']) for q in selection)/len(selection),
            fraction_of_cohort_squared_error=float(r['translation_error_mm'])**2/sse))
    threshold = sorted(float(r['translation_trace_m2']) for r in selection)[math.ceil(len(selection)/2)-1]
    retained = [r for r in selection if float(r['translation_trace_m2'])<=threshold]
    full_rms=math.sqrt(sse/len(selection))
    kept_rms=math.sqrt(sum(float(r['translation_error_mm'])**2 for r in retained)/len(retained))
    check=audit['summaries']['SPX-3']['kimera_front']['near4s']['on']['translation']
    assert math.isclose(full_rms,check['full_rms'],abs_tol=1e-12)
    assert math.isclose(kept_rms, next(d['error_rms'] for d in check['risk'] if d['requested_percent']==50),abs_tol=1e-12)
    supplement=dict(passed=True,inputs_unchanged=True,visual_review='All eight PNG figures inspected',
        spx3_on_kimera_near4s=dict(n=len(selection),retained=len(retained),
            all_rms_mm=full_rms,retained_rms_mm=kept_rms,
            top_three_fraction_squared_error=sum(e['fraction_of_cohort_squared_error'] for e in evidence),
            rows=evidence),
        note='Post-result localization only; no exclusions, recomputed plots, fitted scaling or estimator action.')
    with (ROOT/'supplement.json').open('x') as f: json.dump(supplement,f,indent=2,allow_nan=False)
    files=[ROOT/'PROTOCOL.txt',ROOT/'REPORT.txt',ROOT/'analyze.py',Path(__file__),ROOT/'supplement.json']
    files+=sorted(p for p in OUT.iterdir() if p.is_file())
    with zipfile.ZipFile(ARCHIVE,'x',compression=zipfile.ZIP_DEFLATED,compresslevel=6) as z:
        for p in files:z.write(p,p.relative_to(ROOT))
    with zipfile.ZipFile(ARCHIVE) as z:
        assert z.testzip() is None
        assert len(z.namelist()) == len(files)
    print(json.dumps(dict(passed=True,files=len(files),archive=str(ARCHIVE),bytes=ARCHIVE.stat().st_size,
                         top_three_squared_error_fraction=supplement['spx3_on_kimera_near4s']['top_three_fraction_squared_error'])))


if __name__=='__main__':main()
