import unittest
from analyze_retained import fields, ns, parse_scan, summary


class RetainedParserTest(unittest.TestCase):
    def test_marker_and_ansi(self):
        self.assertIsNone(fields('unrelated', 'ROW,'))
        self.assertEqual(fields('prefix ROW,a,b\x1b[0m\n', 'ROW,'), ['a', 'b'])

    def test_exact_nanoseconds(self):
        self.assertEqual(ns('1727775008.003475000'), 1727775008003475000)

    def test_summary(self):
        s = summary([0, 10, 20])
        self.assertEqual(s['mean'], 10)
        self.assertEqual(s['p95'], 19)
        self.assertEqual(summary([]), {'n': 0})
        with self.assertRaises(ValueError):
            summary([float('nan')])

    def scan(self, total='100'):
        f = ['1727775009.003475000','1','100','10','4','2','0'] + ['0']*16 + [total,'processed']
        f[16], f[17], f[18] = '30', '40', '10'
        return 'GLIM_ODOM_IMU_TIMING_ROW,' + ','.join(f)

    def test_scan_indexes(self):
        s = parse_scan(self.scan())
        self.assertEqual(s['sec'], 1)
        self.assertEqual((s['pre_ms'],s['main_ms'],s['post_ms'],s['total_ms']), (30,40,10,100))

    def test_invalid_scan(self):
        for bad in ('nan','-1'):
            with self.assertRaises(ValueError):
                parse_scan(self.scan(bad))
        with self.assertRaises(ValueError):
            parse_scan('GLIM_ODOM_IMU_TIMING_ROW,too,short')


if __name__ == '__main__':
    unittest.main()
