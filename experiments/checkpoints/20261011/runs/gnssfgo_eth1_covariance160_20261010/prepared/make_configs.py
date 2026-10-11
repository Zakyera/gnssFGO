"""Create two fresh matched configuration files from the validated gate."""
import json
from pathlib import Path
from experiment import ROOT
gate=ROOT.parents[1]/'gnssfgo_eth1_covariance_logging_20261010/gate_ready'
for mode in ['off','on']:
    c=json.loads((gate/'gate_on.json').read_text())
    c['sequence']=f'ETH1 first160s; CBS {mode.upper()}; active-window temporary; native mean alignment; Gaussian WNOA no central IMU'
    c['ros_master_uri']='http://127.0.0.1:11394'
    for s in c['sources']:s['session']='eth1_covariance160_'+mode+'_20261010'
    with (ROOT/('full160_'+mode+'.json')).open('x') as f:json.dump(c,f,indent=2)
