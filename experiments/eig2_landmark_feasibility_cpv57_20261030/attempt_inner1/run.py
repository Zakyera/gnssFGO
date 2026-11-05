"""Preserve failed boundary-search attempt; tighten candidate search, not gate."""
import importlib.util
from pathlib import Path
import traceback

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('cp57', HERE.parent/'analyze.py')
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)
m.H = HERE
original = m.Geometry.feasible


def feasible(self, threshold=3.):
    result = original(self, threshold)
    if threshold==3. and result['outcome']=='unresolved':
        inner = original(self, 3.-1e-6)
        if inner['outcome']=='feasible':
            assert inner['candidate']['passes_geometric_gates']
            return dict(**inner, inner_probe_px=3.-1e-6)
    return result


m.Geometry.feasible = feasible
if __name__=='__main__':
    try:
        m.save('repair_pins.json', {str(p):m.sha(p) for p in
               [Path(__file__), HERE.parent/'PROTOCOL.txt', HERE.parent/'failure.json']})
        m.main()
    except Exception as exc:
        m.save('failure.json', dict(type=type(exc).__name__, error=str(exc),
                                   traceback=traceback.format_exc()))
        raise
