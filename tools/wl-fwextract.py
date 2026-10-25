#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0
"""Extract b43 firmware 832.127 for BCM43227 from Broadcom wl 6.30.223.271.

b43-fwcutter does not know the x86_64 hybrid object of this wl release
(lib/wlc_hybrid.o_shipped in hybrid-v35_64-nodebug-pcoem-6_30_223_271.tar.gz,
installed by broadcom-wl-dkms as /usr/lib/broadcom-wl-dkms/wlc_hybrid.o_shipped).
Like b43-fwcutter, this copies named data objects found through the ELF symbol
table and converts them to the b43 file format. Only the D11 core 30 / N-PHY
rev16 files that b43 validated on BCM43227 are written.

usage: wl-fwextract.py WLC_HYBRID_O OUTDIR
"""

import hashlib
import os
import struct
import sys

WL_MD5 = '6889dbd24abf8006de5cc6eddd138518'

SHT_SYMTAB = 2
SHT_RELA = 4


def elf_objects(blob):
    if blob[:6] != b'\x7fELF\x02\x01':
        raise ValueError('not a little-endian ELF64 object')
    shoff, = struct.unpack_from('<Q', blob, 0x28)
    shentsize, shnum = struct.unpack_from('<HH', blob, 0x3a)
    sections = [struct.unpack_from('<IIQQQQIIQQ', blob, shoff + i * shentsize)
                for i in range(shnum)]
    syms, relocated = {}, {}
    for _, sh_type, _, _, off, size, link, info, _, _ in sections:
        if sh_type == SHT_SYMTAB:
            stroff = sections[link][4]
            for j in range(size // 24):
                name, _, _, shndx, value, symsize = \
                    struct.unpack_from('<IBBHQQ', blob, off + j * 24)
                end = blob.index(b'\0', stroff + name)
                syms[blob[stroff + name:end].decode()] = (shndx, value, symsize)
        elif sh_type == SHT_RELA:
            relocated.setdefault(info, []).extend(
                struct.unpack_from('<Q', blob, off + j * 24)[0]
                for j in range(size // 24))

    def data(name):
        shndx, value, size = syms[name]
        if any(value <= r < value + size for r in relocated.get(shndx, [])):
            raise ValueError(name + ' is relocated')
        start = sections[shndx][4] + value
        return blob[start:start + size]

    return data


def header(fw_type, size):
    return struct.pack('>cBxxI', fw_type, 1, size)


def ucode(data):
    words = struct.unpack('<%dI' % (len(data) // 4), data)
    return header(b'u', len(data)) + struct.pack('>%dI' % len(words), *words)


def initvals(data):
    out, count = [], 0
    for i in range(0, len(data), 8):
        offset, size, value = struct.unpack_from('<HHI', data, i)
        if offset == 0xffff:
            return header(b'i', count) + b''.join(out)
        if offset >= 0x1000 or size not in (2, 4) or value >> (8 * size):
            raise ValueError('bad initval at %#x' % offset)
        if size == 4:
            out.append(struct.pack('>HI', offset | 0x8000, value))
        else:
            out.append(struct.pack('>HH', offset, value))
        count += 1
    raise ValueError('unterminated initvals')


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__.strip().splitlines()[-1])
    blob = open(sys.argv[1], 'rb').read()
    if hashlib.md5(blob).hexdigest() != WL_MD5:
        sys.exit('%s: not the wl 6.30.223.271 x86_64 object' % sys.argv[1])
    data = elf_objects(blob)
    size, = struct.unpack('<I', data('d11ucode30_mimosz'))
    code = data('d11ucode30_mimo')
    if size != len(code):
        sys.exit('ucode size mismatch')
    files = {
        'ucode30_mimo.fw': ucode(code),
        'n16initvals30.fw': initvals(data('d11n16initvals30')),
        'n16bsinitvals30.fw': initvals(data('d11n16bsinitvals30')),
    }
    os.makedirs(sys.argv[2], exist_ok=True)
    for name, contents in files.items():
        with open(os.path.join(sys.argv[2], name), 'wb') as f:
            f.write(contents)
        print(hashlib.sha256(contents).hexdigest(), name)


if __name__ == '__main__':
    main()
