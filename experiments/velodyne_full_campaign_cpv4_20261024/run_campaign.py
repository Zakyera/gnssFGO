"""Finite sequential campaign, one attempt per pair; no estimator changes/retries."""
from pathlib import Path
import datetime,fcntl,json,os,shutil,subprocess,sys,time
H=Path(__file__).resolve().parent
CONTAINERS=['cbsms_ws_gpu','ubuntu-jammy-humble-nvidia']

def now():return datetime.datetime.now().astimezone().isoformat()
def save(name,value):
 with (H/name).open('x') as f:json.dump(value,f,indent=2)
def update(stage,**extra):
 value=dict(stage=stage,updated=now(),pid=os.getpid(),**extra)
 tmp=H/'status.pending';tmp.write_text(json.dumps(value,indent=2)+'\n');tmp.replace(H/'status.json')
 print(json.dumps(value),flush=True)
def container_state():
 return subprocess.check_output(['docker','inspect','-f','{{.State.Status}}',*CONTAINERS],text=True).split()

def report(config,outcomes):
 lines=['CP-V4 FULL VELODYNE CAMPAIGN — '+now(),
  'Matched original/native GLIM, original/native Kimera, concurrent ONLINE WNOA fusion.',
  '0.20x, frozen CP-V2 settings. Full-saved metrics vs shared-CPT7 IE-TC, not independent prism GT.',
  'DELAYED fixed-lag-finalized fusion is separate; not online/full-history.',
  'Startup/tail/keyframe gaps and common-mask scores are in each sequence RESULTS_REPORT.txt.',
  '', 'dataset role mode ATE_m tRPE1s_m rRPE1s_deg path_prism_m span_s rows']
 for cfg in config['sequences']:
  slug=cfg['slug'];item=outcomes.get(slug,{});path=H/slug/'results.json'
  if item.get('state')!='complete' or not path.exists():
   lines.append(cfg['dataset']+' NOT_COMPLETE '+json.dumps(item));continue
  data=json.loads(path.read_text())
  for role in ('native_glim','native_kimera','fused_online','fused_finalized'):
   for mode in ('off','on'):
    r=data['evaluation']['results'][mode+'_'+role]['full_saved']
    lines.append(f"{cfg['dataset']} {role} {mode} {r['ATE_RMSE_m']:.6f} {r['translation_RPE_1s_RMSE_m']:.6f} {r['rotation_RPE_1s_RMSE_deg']:.6f} {r['path_length_prism_m']:.6f} {r['span_s']:.6f} {r['rows']}")
 lines+=['','UNAVAILABLE INPUTS',json.dumps(config['unavailable'],indent=2),'',
  'All raw covariance/revision/payload/timing histories are retained in per-sequence prepared roots.',
  'No weights/model/calibration tuning; no nonlinear stationarity/calibrated uncertainty claim.',
  'Read per-sequence terminal omissions, prior-absorbed revision expiry and coverage limitations.',
  'Existing mean-only source-to-source covariance alignment approximation remains unchanged.']
 with (H/'RESULTS_REPORT.txt').open('x') as f:f.write('\n'.join(lines)+'\n')

def main():
 lock=(H/'campaign.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
 assert not (H/'start.json').exists() and not (H/'status.json').exists(),'One launch only'
 config=json.loads((H/'campaign.json').read_text());outcomes={}
 assert container_state()==['exited','exited'],'Refuse to adopt active containers'
 save('start.json',dict(started=now(),pid=os.getpid(),initial_containers=['exited','exited']))
 failure=None
 try:
  for cfg in config['sequences']:
   slug=cfg['slug'];pair=H/slug
   if not (pair/'prepared.json').exists():
    assert (pair/'blocked.json').exists(),('unreviewed preparation missing',slug)
    outcomes[slug]=dict(state='blocked',evidence=str(pair/'blocked.json'));continue
   pre=json.loads((pair/'prepared.json').read_text());assert pre['passed']
   assert not (pair/'status.json').exists() and not (pair/'start.json').exists(),('refuse duplicate',slug)
   assert container_state()==['exited','exited'],'Previous jobs not cleaned up'
   free=shutil.disk_usage(H).free;duration=pre['window']['duration_s']
   # CP-V2 actual pair453853184bytes/30s, plus20% headroom.
   estimate=453853184/30*duration*1.2
   assert free-estimate/2>8*1024**3 and free-estimate>5*1024**3,('projected pair reserve',slug,free,estimate)
   who=slug.upper()+'-CPV4'
   cmd=['systemd-inhibit','--what=sleep:idle:handle-lid-switch','--mode=block',
        '--who='+who,'--why=Authorized full matched Velodyne experiment',
        'gnome-session-inhibit','--app-id',who,'--reason','Authorized full matched Velodyne experiment',
        '--inhibit','suspend:idle','python3',str(pair/'run_pair.py')]
   update('running_pair',sequence=slug,outcomes=outcomes,free_bytes=free,forecast_pair_bytes=estimate)
   with (pair/'controller.log').open('x') as log:
    proc=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT)
    save(slug+'_controller.json',dict(pid=proc.pid,command=cmd,started=now()))
    rc=proc.wait()
   state=json.loads((pair/'status.json').read_text()) if (pair/'status.json').exists() else {}
   closed=json.loads((pair/'closeout.json').read_text()) if (pair/'closeout.json').exists() else {}
   if rc==0 and state.get('state')=='complete' and closed.get('passed'):
    assert json.loads((pair/'results.json').read_text())['passed']
    outcomes[slug]=dict(state='complete',finished=now(),report=str(pair/'RESULTS_REPORT.txt'))
   else:
    outcomes[slug]=dict(state='failed',returncode=rc,finished=now(),evidence=state)
    assert closed.get('passed') and not state.get('cleanup_error'),('unsafe cleanup/integrity failure',slug,state)
    assert container_state()==['exited','exited']
    # Case-level estimator/input failure: retain prefix, no ON if OFF failed,
    # no automatic retry. Independent remaining missions may still be useful.
   update('pair_finished',sequence=slug,outcomes=outcomes)
  assert container_state()==['exited','exited']
  subprocess.run(['python3',str(H.parent/'checkpoints/20261023_hesai_four_sequences/verify_checkpoint.py'),'--originals'],check=True)
 except BaseException as exc:
  failure=repr(exc);save('failed.json',dict(error=failure,at=now(),outcomes=outcomes))
 finally:
  report(config,outcomes)
  save('completion.json',dict(finished=now(),outcomes=outcomes,fatal_error=failure,
    all_prepared_pairs_complete=all(outcomes.get(c['slug'],{}).get('state')=='complete' for c in config['sequences'] if (H/c['slug']/'prepared.json').exists()),
    free_bytes=shutil.disk_usage(H).free))
  update('finished' if failure is None else 'blocked',outcomes=outcomes,error=failure)
  with (H/'REPORT.txt').open('a') as f:f.write('\nCampaign stopped '+now()+'\n'+json.dumps(dict(outcomes=outcomes,error=failure),indent=2)+'\nSee RESULTS_REPORT.txt.\n')
 if failure:raise SystemExit(1)

if __name__=='__main__':main()
