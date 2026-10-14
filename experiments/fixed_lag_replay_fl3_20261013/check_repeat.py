import json,sys
from driver import ROOT
from accept_prefix import files
if sys.argv[1]=='prefix':
    a,b='prefix_gate','prefix_timed';destination='timed_prefix_acceptance.json'
else:
    a,b='full_timed','full_repeat';destination='full_repeat_validation.json'
assert json.loads((ROOT/(b+'.json')).read_text())['returncode']==0
left=files(ROOT/(a+'_outputs'));right=files(ROOT/(b+'_outputs'))
result=dict(passed=left==right,compared_files=len(left),archive_bytes_identical=left==right,
            first=a,second=b)
with (ROOT/destination).open('x') as f:json.dump(result,f,indent=2)
print(json.dumps(result,indent=2))
assert result['passed']
