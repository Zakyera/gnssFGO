"""Fail-closed check of the actual libraries mapped by the GLIM process."""

from pathlib import Path
import sys


REQUIRED = (
    'libglim_ros.so',
    'libgtsam_points.so.1.2.1',
    'libglim.so',
    'libcbs.so',
    'libe1b_profile.so',
    'libgtsam.so.4.3a1',
    'libgtsam_points_cuda.so.1.2.1',
)


def mapped_paths(maps_text):
    paths = set()
    for line in maps_text.splitlines():
        fields = line.split(maxsplit=5)
        if len(fields) == 6 and fields[5].startswith('/'):
            paths.add(fields[5])
    return paths


def require_glim_libraries(maps_text, library_dir):
    """Require the unique mapped path for every pinned library's real target."""
    library_dir = Path(library_dir)
    paths = mapped_paths(maps_text)
    expected = {}
    for name in REQUIRED:
        target = str((library_dir / name).resolve(strict=True))
        actual = {path for path in paths if Path(path).name == name}
        if actual != {target}:
            raise AssertionError((name, target, sorted(actual)))
        expected[name] = target
    return expected


def self_test(library_dir, fixture_path):
    maps_text = Path(fixture_path).read_text()
    expected = require_glim_libraries(maps_text, library_dir)
    assert expected['libglim_ros.so'] == str(Path(library_dir) / 'libglim_ros.so')
    for name, target in expected.items():
        changed = maps_text.replace(target, '/tmp/unexpected/' + name)
        assert changed != maps_text, name
        try:
            require_glim_libraries(changed, library_dir)
        except AssertionError:
            pass
        else:
            raise AssertionError('Wrong library accepted: ' + name)
    print('MAP_GUARD_PASS', len(expected), 'real libraries,', len(expected),
          'wrong-path negatives')


if __name__ == '__main__':
    if len(sys.argv) != 3:
        raise SystemExit('usage: map_guard.py LIBRARY_DIR RETAINED_MAPS')
    self_test(Path(sys.argv[1]), Path(sys.argv[2]))
