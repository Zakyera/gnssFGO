"""Report retained observations only; no estimator runs, fitting, or GT tuning."""
import csv,json
from driver import ROOT,BASEROOT,HERE,protected,sha

def read_csv(path):
    with path.open() as f:return list(csv.DictReader(f))

def main():
    result={}
    for name,root in [('before',BASEROOT),('after',ROOT)]:
        r=read_csv(root/'full_service.csv');pub=[v for v in r if int(v['nonempty'])]
        timing=json.loads((root/'full_timing_analysis.json').read_text())
        result[name]={
          'slots_at_support':{i:next(int(v['slots']) for v in pub if int(v['newest'])==i) for i in (70,200,600,1200,1224,1480)},
          'peak':{k:max(int(v[k]) for v in pub) for k in ('slots','nonempty','supports','fixed_keys','rss_kib')},
          'wall_seconds':timing['process_wall_seconds'],
          'whole_support_seconds':timing['whole_support_seconds'],
          'late_seconds':timing['bins']['late']['total'],
          'queue_peak':timing['simulated_queue_max'],
          'latency_max':timing['simulated_latency_seconds']['max'],
          'timing_target_pass':timing['operational_target_pass']}
    validation=json.loads((ROOT/'full_validation.json').read_text())
    result['state_equivalence']=validation['fl3h_state_equivalence']
    result['numerical_gate_pass']=validation['fl3h_numerical_gate_pass']
    result['source_accounting_and_payload_pass']=validation['source_accounting_and_payload_pass']
    assert result['after']['peak']['slots']<=200
    result['scope']='one existing148.58s saved arrival capture; not co-running or GT accuracy evidence'
    result['protected_unchanged']=protected()==json.loads((ROOT/'protected.json').read_text())
    assert result['protected_unchanged']
    result['protected_count']=len(protected());result['binary_sha256']=sha(ROOT/'build_replay')
    result['source_hashes']={p.name:sha(p) for p in HERE.iterdir() if p.suffix in ('.cpp','.h','.py')}
    with (ROOT/'summary.json').open('x') as f:json.dump(result,f,indent=2)
    print(json.dumps({k:v for k,v in result.items() if k!='source_hashes'},indent=2))

if __name__=='__main__':main()
