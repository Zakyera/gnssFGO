"""Fail-closed regression gate for this exact x86-64/GCC private wrapper build.

This is not a general C++ ABI checker: it validates the immediate allocation
preceding the unique GlimCbsBridge constructor relocation. A changed compiler
pattern must be inspected, not silently accepted.
"""
import re
import subprocess


def allocation(disassembly):
    lines = disassembly.splitlines()
    hits = [i for i, line in enumerate(lines)
            if 'R_X86_64_' in line and
            'glim::GlimCbsBridge::GlimCbsBridge(ros::NodeHandle&)' in line]
    if len(hits) != 1:
        raise ValueError('Expected one bridge constructor relocation')
    end = hits[0]
    region = lines[max(0, end - 14):end + 1]
    new_calls = [i for i, line in enumerate(region)
                 if 'operator new(unsigned long)' in line]
    if len(new_calls) != 1:
        raise ValueError('Expected one preceding allocation')
    window = region[max(0, new_calls[0] - 3):new_calls[0]]
    sizes = re.findall(r'mov\s+\$0x([0-9a-f]+),%edi', '\n'.join(window))
    if len(sizes) != 1:
        raise ValueError('Cannot establish immediate allocation size')
    return int(sizes[0], 16), '\n'.join(region)


def require_size(obj, expected):
    text = subprocess.check_output(['objdump', '-drC', str(obj)], text=True)
    size, excerpt = allocation(text)
    if size != expected:
        raise ValueError(f'Layout mismatch: allocator={size}, header={expected}')
    return excerpt
