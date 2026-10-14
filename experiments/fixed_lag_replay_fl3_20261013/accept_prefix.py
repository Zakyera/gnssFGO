"""Read-only acceptance of unchanged prefix and observer-only repeat."""
import json,re,subprocess
from driver import ROOT,PREV,digest,protected
def files(root):return {str(p.relative_to(root)):digest(p) for p in root.rglob('*') if p.is_file()}
def main():
    a=json.loads((ROOT/'prefix_audit.json').read_text());assert a['status']=='PREFIX_NUMERICAL_AND_ACCOUNTING_PASS'
    assert files(ROOT/'prefix_gate_outputs')==files(ROOT/'prefix_observe_outputs')
    assert a['warning_keys']==[[k,i] for i in range(a['retired_supports']) for k in ('x','v','w')]
    log=(ROOT/'prefix_observe.log').read_text();assert log.count('FL3_FIRST_MISSING_KEY_CALL_STACK')==1
    addresses=re.findall(r'build_wire3\(\+(0x[0-9a-f]+)\)',log)
    stack=subprocess.check_output(['addr2line','-Cf','-e',str(PREV/'build_wire3'),*addresses],text=True)
    assert stack.splitlines()[0]=='fl2a::BoundarySmoother::adoptCharts()'
    result=dict(status='PREFIX_ACCEPTED_FOR_FULL_REPLAY',accounting_and_numerical_pass=True,
      diagnostic_repeat_archive_byte_identical=True,warning_count=a['runtime_warning_count'],decoded_stack=stack,
      warning_scope='only eliminated X/V/W keys; transitional calculateBestEstimate in adoptCharts before retained-graph reload',
      reasoning='Pinned VectorValues::update skips missing destination keys. These are eliminated leaf frontals, not surviving values. Conditional back-substitution cannot feed a leaf solution into its parents. Retained graph is then reloaded and all publications pass the unchanged chart-correct objective check. Warnings are not suppressed; do not claim library-wide or crash-resume correctness.',
      original_exclusions=25,artificial_prefix_EOF_pending_exclusion=1,additional_lag_exclusions=0,
      protected_unchanged=protected()==json.loads((ROOT/'protected.json').read_text()))
    with (ROOT/'prefix_acceptance.json').open('x') as f:json.dump(result,f,indent=2)
    print(json.dumps(result,indent=2))
if __name__=='__main__':main()
