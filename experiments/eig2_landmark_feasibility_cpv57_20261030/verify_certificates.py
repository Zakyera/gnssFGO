"""Check that each dual-certificate row is a valid necessary camera constraint.

This is independent of the cutting-plane indexing/generation code. Also recheck
the weighted contradiction with finite residual and the necessary point box.
"""
import importlib.util
import json
from pathlib import Path
import numpy as np

H=Path(__file__).resolve().parent
s=importlib.util.spec_from_file_location('cp57',H/'analyze.py')
m=importlib.util.module_from_spec(s);s.loader.exec_module(m)


def main():
    findings=json.loads((H/'findings.json').read_text())
    gs={(r['phase'],r['slot'],r['transform']):r for line in m.GEOMETRY.open() if (r:=json.loads(line))}
    row_count=0;cert_count=0;margins=[];residuals=[]
    for r in findings['records']:
        cert=r['certificate']
        if not cert:continue
        g=m.Geometry(gs[(r['phase'],r['slot'],r['transform'])])
        A=np.array(cert['constraint_rows']);b=np.array(cert['constraint_rhs'])
        for a,bb in zip(A,b):
            candidates=[]
            # Necessary coordinate bounds from <=10m to first camera.
            for d in np.r_[np.eye(3),-np.eye(3)]:
                candidates.append(np.linalg.norm(a-d)+abs(bb-10))
            for i in range(len(g.t)):
                # Nonnegative depth.
                candidates.append(np.linalg.norm(a+g.P[i,2])+abs(bb-g.offset[i,2]))
                # A tangent direction to the circular reprojection bound.
                u=np.linalg.lstsq(g.N[i].T,a+3*g.P[i,2],rcond=None)[0]
                candidates.append(np.linalg.norm(a-(u@g.N[i]-3*g.P[i,2]))
                                  +abs(bb-(3*g.offset[i,2]-u@g.no[i]))
                                  +abs(np.linalg.norm(u)-1))
                # Necessary supporting plane to the 10m distance ball.
                candidates.append(abs(np.linalg.norm(a)-1)+abs(bb-(10+a@g.t[i])))
            residual=float(min(candidates));assert residual<1e-7,(r['slot'],residual)
            row_count+=1;residuals.append(residual)
        y=np.array(cert['nonnegative_weights']);assert np.all(y>=0)
        margin=float(-y@b-10*np.sum(np.abs(y@A)))
        assert margin>1e-8
        cert_count+=1;margins.append(margin)
    assert cert_count==75
    m.save('certificate_verification.json',dict(passed=True,certificates=cert_count,
           independently_recognized_necessary_halfspaces=row_count,
           max_row_recognition_residual=max(residuals),minimum_contradiction_margin=min(margins),
           findings_sha256=m.sha(H/'findings.json'),geometry_sha256=m.sha(m.GEOMETRY),
           qualification='Numerically verified at original3px/10m/depth constraints, not formal exact-arithmetic proof.'))
    print('PASS',cert_count,'certificates;',row_count,'necessary halfspaces; min margin',min(margins))


if __name__=='__main__':main()
