#!/usr/bin/env python3
"""Focused transport tests; no real estimator is launched."""
import copy
import tempfile
import unittest
from pathlib import Path

import numpy as np
from live_receiver_protocol import LineTail, Versions, parse_belief, parse_online_pose, validate_config


class ProtocolTests(unittest.TestCase):
    def test_model_independent_pose_publication(self):
        prefix = 'state_index,timestamp_ns,px,py,pz,qx,qy,qz,qw,vx,vy,vz'
        for tail in ['bax,bay,baz,bgx,bgy,bgz', 'wx,wy,wz',
                     'wx,wy,wz,alpha_x,alpha_y,alpha_z,a_x,a_y,a_z']:
            header = (prefix+','+tail).split(',')
            row = ['0','1000000000','1','2','3','0','0','0','1']+['0']*(len(header)-9)
            self.assertEqual(parse_online_pose(header, ','.join(row)),
                             (1000000000,[1.,2.,3.],[0.,0.,0.,1.]))
            with self.assertRaises(ValueError):
                parse_online_pose(header, ','.join(row[:-1]))
            row[2]='nan'
            with self.assertRaises(ValueError):
                parse_online_pose(header, ','.join(row))

    def setUp(self):
        self.mu = np.arange(6)*0.001
        self.cov = np.eye(6)*0.01+np.ones((6, 6))*0.0001
        self.mu_token = ';'.join(map(str, self.mu))
        self.cov_token = ';'.join(':'.join(map(str, row)) for row in self.cov)
        self.g = 'GLIM_CBS_EXPORT_BELIEF_ROW,g,2,4,0,2.5,9,1.0,2.0,'+self.mu_token+','+self.cov_token

    def test_exact_dense_payload(self):
        b = parse_belief('glim', self.g)
        np.testing.assert_array_equal(b['mean'], self.mu)
        np.testing.assert_array_equal(np.asarray(b['covariance']).reshape(6, 6), self.cov)
        fields = ['8', 'k', '2', '4', '1.0', '2.0', '0', 'first']+['0']*14+list(map(str, self.mu))+list(map(str, self.cov.flat))
        k = parse_belief('kimera', 'KIMERA_CBS_REVISION_EDGE_ROW,'+','.join(fields))
        self.assertEqual(k['mean'], b['mean'])
        self.assertEqual(k['covariance'], b['covariance'])

    def test_reindex_keep_changed_firsts_duplicate_dedup(self):
        v = Versions()
        b = parse_belief('glim', self.g)
        first = v.accept('sensor', 'run1', b, 2500000000)
        self.assertEqual(first[5], 0)
        self.assertIsNone(v.accept('sensor', 'run1', b, 2600000000))
        b['mean'][0] += 0.01
        self.assertEqual(v.accept('sensor', 'run1', b, 2700000000)[5], 1)
        self.assertEqual(v.accept('sensor', 'run2', b, 2800000000)[5], 0)
        b['ti'] += 1
        with self.assertRaises(ValueError):
            v.accept('sensor', 'run1', b, 2900000000)

    def test_invalid_never_silently_repaired(self):
        b = parse_belief('glim', self.g)
        with self.assertRaises(ValueError):
            Versions().accept('sensor', 'run', b, 1000000000)
        with self.assertRaises(ValueError):
            parse_belief('glim', 'GLIM_CBS_REVISION_ERROR_ROW,error')
        with self.assertRaises(ValueError):
            parse_belief('glim', self.g.replace('0.0101', '-0.0101'))

    def test_partial_line_and_truncation(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d)/'log'
            p.write_bytes(b'hello')
            tail = LineTail(p)
            self.assertEqual(tail.read(), [])
            with p.open('ab') as f:
                f.write(b' world\nnext')
            self.assertEqual(tail.read(), ['hello world'])
            p.write_bytes(b'')
            with self.assertRaises(RuntimeError):
                tail.read()
            tail.file.close()

    def test_arbitrary_source_combinations(self):
        # These are examples, not baked-in cardinalities or a capacity claim.
        for kinds in [('glim',), ('kimera',), ('glim', 'kimera'), ('kimera',)*5+('glim',)*2]:
            c = dict(sequence='test', source_protocol='classic', ros_master_uri='x',
                     imu_topic='imu', imu_frame='cpt7_imu', sources=[
                         dict(id='s'+str(i), session='one', kind=k, log_glob='/unique/'+str(i),
                              belief_body_frame='cpt7_imu')
                         for i, k in enumerate(kinds)])
            validate_config(c)
            c['receiver_model'] = 'source_wnoj'
            validate_config(c)
            c['receiver_model'] = 'source_wnoa'
            with self.assertRaises(ValueError):
                validate_config(c)
            c['receiver_environment'] = dict(WNOJ_STRICT_PUBLICATION='1',
                                           WNOJ_INCREMENTAL_PROFILE='nominal')
            validate_config(c)
            c['receiver_environment']['AUDIT_RECORD_ALL'] = '1'
            with self.assertRaises(ValueError):
                validate_config(c)
            c.pop('receiver_environment')
            c['receiver_model'] = 'invalid'
            with self.assertRaises(ValueError):
                validate_config(c)
            c.pop('receiver_model')
        c['sources'].append(copy.deepcopy(c['sources'][0]))
        with self.assertRaises(ValueError):
            validate_config(c)


if __name__ == '__main__':
    unittest.main()
