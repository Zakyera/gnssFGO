import tempfile
import unittest
from pathlib import Path

import numpy as np

from inspect_capture import covariance_stats, journal_inventory, matrix_token


class Tests(unittest.TestCase):
    def test_dense_covariance_preserved(self):
        a = np.arange(36, dtype=float).reshape(6, 6) / 100
        c = a @ a.T + np.eye(6)
        text = ";".join(":".join(format(x, ".17g") for x in row) for row in c)
        np.testing.assert_array_equal(matrix_token(text), c)
        result = covariance_stats(c)
        self.assertEqual(result["rotation_trace_rad2"], np.trace(c[:3, :3]))
        self.assertEqual(result["rho_trace_m2"], np.trace(c[3:, 3:]))

    def test_invalid_matrices_rejected(self):
        with self.assertRaises(ValueError):
            matrix_token("1;2;3")
        for bad in [np.full((6, 6), np.nan), np.zeros((6, 6)),
                    np.diag([1, 1, 1, 1, 1, -1]), np.eye(6) + np.triu(np.ones((6, 6)), 1)]:
            with self.subTest(matrix=bad), self.assertRaises(ValueError):
                covariance_stats(bad)

    def event(self, rev=0, scale=1, arrival=30):
        return ",".join(map(str, ["B", arrival, "glim", "session", 0, rev,
                                  0, 2, 10, 20, *([0] * 6),
                                  *(np.eye(6) * scale).ravel()])) + "\n"

    def inventory(self, text):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "events.csv"
            path.write_text(text)
            return journal_inventory(path)

    def test_revisions_not_independent_ids(self):
        r = self.inventory(self.event() + self.event(1, .5, 31))
        s = r["sources"]["glim"]
        self.assertEqual(s["transported_ids"], 1)
        self.assertEqual(s["versions"], 2)
        self.assertEqual(s["last_observed"]["rho_trace_m2"]["median"], 1.5)

    def test_missing_revision_and_future_endpoint_rejected(self):
        for text in [self.event(1), self.event() + self.event(2), self.event(arrival=19),
                     self.event() + self.event(1, arrival=29)]:
            with self.subTest(text=text), self.assertRaises(ValueError):
                self.inventory(text)


if __name__ == "__main__":
    unittest.main()
