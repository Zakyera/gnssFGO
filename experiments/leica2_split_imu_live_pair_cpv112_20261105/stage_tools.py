"""Derive NEW mission-only tools from sealed CP111; no estimator execution."""
import ast,difflib,hashlib,json
from paths import *
def once(s,a,b):
    assert s.count(a)==1,(a,s.count(a));return s.replace(a,b)
def digest(p):
    with p.open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
seal=json.loads((BASE_H/'complete.json').read_text());assert seal['passed']
assert json.loads((BASE_H/'post_closeout.json').read_text())['passed']
for p,d in seal['output_sha256'].items():assert digest(Path(p))==d,p
assert not P.exists() and not (H/'available_data_review.json').exists()
mapping=[('SPX-3','LEICA-2'),('SPX3','LEICA2'),('2024-11-02-17-43-10','2024-11-25-16-36-19'),
 ('1730565899872888651','1732549034915071665'),('1730566096892888651','1732549463665071665'),
 ('29000000000','6000000000'),('offset_s=29.0','offset_s=6.0'),('197.02','428.75'),
 ('==1970','==4287'),('==1971','==4288'),('==19702','==42875'),(')*1e-9>196',')*1e-9>427.73'),
 ('spx3_stationary_full_cpk28_20261023','leica2_full_cpk29_20261023')]
rows=[];diff=[]
for name in ('inspect_available.py','audit_inputs.py','prepare.py'):
    old=(BASE_H/name).read_text();s=old
    if name!='prepare.py':
        # Longer integer literals first to avoid a partial count replacement.
        for a,b in sorted(mapping,key=lambda x:-len(x[0])):s=s.replace(a,b)
    if name=='audit_inputs.py':
        s=once(s,"starts.append(r.start_time)","starts.extend(next(r.messages(connections=[c]))[1] for c in r.connections if c.topic in wanted)")
        s=once(s,"assert min(starts)+6000000000==epoch,'New IMU must not silently change rosbag playback epoch'", "assert min(starts)==1732549028915071665 and min(starts)+6000000000==epoch,'Selected-topic epoch changed with new Alpha input'")
    if name=='prepare.py':
        start=s.index('replacements=[');end=s.index('records=[];diff=[]',start)
        s=s[:start]+'''replacements=[(BASE_H.name,H.name),('2024-11-02-17-43-10','2024-11-25-16-36-19'),
 ('SPX-3','LEICA-2'),('SPX3-CPV111','LEICA2-CPV112'),('SPX3','LEICA2'),
 ('spx3_cpv111_','leica2_cpv112_'),('CP-V111','CP-V112'),('11641','11651'),('11642','11652'),
 ('1730565898','1732549034'),('197020000000','428750000000'),('197.02','428.75'),('195.02','426.75')]
def rebase(s):
    for a,b in replacements:s=s.replace(a,b)
    s=re.sub(r'(?<![\\w.])29\\.0(?![\\w.])','6.0',s)
    for a,b in [('1970','4287'),('1967','4284'),('1950','4267')]:
        s=re.sub(r'(?<!\\d)'+a+r'(?!\\d)',b,s)
    return s
'''+s[end:]
        start=s.index("    if name=='preflight_tests.py':");end=s.index('    ast.parse(s)',start)
        s=s[:start]+'''    if name=='preflight_tests.py':
        s=once(s,"gnssfgo_spx3_stationary_full_cpk28_20261023","gnssfgo_leica2_full_cpk29_20261023")
    elif name=='admission.py':
        s=once(s,"forecast_bytes=16*2**30","forecast_bytes=20*2**30")
        s=once(s,"required_free_bytes=21*2**30","required_free_bytes=25*2**30")
        s=once(s,"16GiB full-pair allowance exceeds CP95 9.45GiB forecast, plus5GiB reserve; LEICA2 window shorter than CP108; retain conservative16GiB allowance",
                  "20GiB full-pair allowance scales CP108 16GiB by LEICA2/EIG2 duration ratio1.208, plus5GiB reserve")
'''+s[end:]
        s=s.replace('Fresh mechanical CP108->SPX3','Fresh mechanical CP111->LEICA2').replace("ports=[11641,11642]","ports=[11651,11652]").replace('Prepared fresh SPX3','Prepared fresh LEICA2')
    ast.parse(s)
    with (H/name).open('x') as f:f.write(s)
    rows.append(dict(name=name,source=str(BASE_H/name),source_sha256=digest(BASE_H/name),sha256=digest(H/name)))
    diff.extend(difflib.unified_diff(old.splitlines(True),s.splitlines(True),fromfile=str(BASE_H/name),tofile=str(H/name)))
with (H/'tool_derivation.json').open('x') as f:json.dump(rows,f,indent=2)
with (H/'tool_derivation.diff').open('x') as f:f.write(''.join(diff))
print('Staged NEW LEICA2 input/preparation tools. No source/receiver execution.')
