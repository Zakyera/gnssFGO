"""Exact coverage_control.py schemas. Booleans/coercions are not counts."""
def validate_injections(mode, value):
    assert mode in ('off', 'on'), mode
    if mode == 'off':
        assert type(value) is int and value == 0, ('OFF requires integer zero', value)
        return {'total': 0}
    keys = {'G2K_queued', 'G2K_persistent', 'K2G_persistent'}
    assert type(value) is dict and set(value) == keys, ('ON direction schema', value)
    assert all(type(v) is int and v > 0 for v in value.values()), ('ON positive integer counts', value)
    assert value['G2K_queued'] == value['G2K_persistent'], ('G2K queue/insertion conservation', value)
    return dict(value)

