"""Resolve quantization caveat and reconstruct initialization packet only."""
import json
from pathlib import Path
import numpy as np
from analyze import H, P85, pin, load, raw_imu, save, PINS, sha, AnyReader

assert not (H/'supplement_2.json').exists()
j=load(P85/'input_audit.json'); epoch=j['epoch_ns']
A=raw_imu(j['bag_files']['alphasense_imu']['path'],epoch)
C=raw_imu(j['bag_files']['cpt7_imu']['path'],epoch)
x=A[:,4:]/(39.2/32768)
out={'quantization_caveat': 'Supersedes the overly broad interpretation string in supplement_1.json: pure unaveraged signed16 quantization is NOT supported. The values are mostly on a tenth-count grid; averaging is a hypothesis, not verified firmware behavior. Two -39.2 values suggest clipping, but actual range/configuration remains unconfirmed.',
     'quantization_tests': []}
for n in [1,2,4,5,10,20,100]:
    e=np.abs(n*x-np.rint(n*x))
    out['quantization_tests'].append({'count_subdivisions':n,'fraction_integer_within_1e_7':float((e<1e-7).mean()),'max_error_counts':float(e.max())})
camera_path=pin(j['bag_files']['alphasense']['path'])
stamps=[]
with AnyReader([camera_path]) as reader:
    cs=[c for c in reader.connections if c.topic=='/boxi/alphasense/front_left/image_raw/compressed']
    assert len(cs)==1
    for c,t,b in reader.messages(connections=cs,start=epoch,stop=epoch+400000000):
        m=reader.deserialize(b,c.msgtype)
        stamps.append(m.header.stamp.sec*10**9+m.header.stamp.nanosec)
assert stamps[3]==1727775008155395209
lo=(stamps[2]-epoch)*1e-9;hi=(stamps[3]-epoch)*1e-9
out['initial_packet_reconstruction']={'camera_stamps_ns':stamps,'interval_s':[lo,hi],
    'method':'Code-equivalent border interpolation and unweighted column mean of first valid input packet; reconstructed from saved raw data, not an original saved internal bias trace.',
    'arms':{}}
for name,a in [('Alphasense',A),('CPT7',C)]:
    inner=a[(a[:,0]>lo)&(a[:,0]<hi),1:]
    low=np.array([np.interp(lo,a[:,0],a[:,k]) for k in range(1,7)])
    high=np.array([np.interp(hi,a[:,0],a[:,k]) for k in range(1,7)])
    mean=np.vstack([low,inner,high]).mean(axis=0)
    c=-mean[5]/np.linalg.norm(mean[3:]); identity=abs(1-c)<1e-3
    assert identity
    out['initial_packet_reconstruction']['arms'][name]={'samples':len(inner)+2,
        'mean_acc_m_s2':mean[3:].tolist(),'mean_gyro_rad_s':mean[:3].tolist(),
        'gravity_angle_deg':float(np.rad2deg(np.arccos(c))),
        'identity_rotation_branch':bool(identity),
        'initial_acc_bias_m_s2':(mean[3:]+[0,0,9.81]).tolist(),
        'initial_gyro_bias_rad_s':mean[:3].tolist()}
for f in ['src/dataprovider/DataProviderModule.cpp','src/utils/ThreadsafeImuBuffer.cpp',
          'src/frontend/StereoVisionImuFrontend.cpp','src/pipeline/StereoImuPipeline.cpp',
          'include/kimera-vio/initial/InitializationFromImu.h']:
    pin(Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/Kimera-VIO')/f)
pin(Path(__file__));assert all(sha(p)==s for p,s in PINS.items())
save(H/'supplement_2_inputs.json',PINS);save(H/'supplement_2.json',out)
print(json.dumps(out,indent=2))
