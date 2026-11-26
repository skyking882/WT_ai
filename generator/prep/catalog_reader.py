"""Read local VROMFS and dictionary-compressed BLK metadata for map selection."""
from pathlib import Path
import ctypes
import os
import struct


class Span(ctypes.Structure):
    _fields_ = [("data", ctypes.c_void_p), ("size", ctypes.c_size_t)]


class Codecs:
    def __init__(self, directory):
        self.directory = os.add_dll_directory(str(Path(directory).resolve()))
        self.lib = ctypes.CDLL(str(Path(directory).resolve() / "daKernel-dev.dll"))
        self.zstd = getattr(self.lib, "?zstd_decompress@@YA_KPEAX_KPEBX1@Z")
        self.oodle = getattr(self.lib, "?oodle_decompress@@YA_KPEAX_KPEBX1@Z")
        for f in (self.zstd, self.oodle):
            f.argtypes = (ctypes.c_void_p, ctypes.c_size_t, ctypes.c_void_p, ctypes.c_size_t)
            f.restype = ctypes.c_size_t
        self.create_dict = getattr(self.lib, "?zstd_create_ddict@@YAPEAUZSTD_DDict_s@@V?$Span@$$CBD@dag@@_N@Z")
        self.create_dict.argtypes = (Span, ctypes.c_bool)
        self.create_dict.restype = ctypes.c_void_p
        self.create_ctx = getattr(self.lib, "?zstd_create_dctx@@YAPEAUZSTD_DCtx_s@@_N@Z")
        self.create_ctx.argtypes = (ctypes.c_bool,)
        self.create_ctx.restype = ctypes.c_void_p
        self.with_dict = getattr(self.lib, "?zstd_decompress_with_dict@@YA_KPEAUZSTD_DCtx_s@@PEAX_KPEBX2PEBUZSTD_DDict_s@@@Z")
        self.with_dict.argtypes = (ctypes.c_void_p, ctypes.c_void_p, ctypes.c_size_t,
                                  ctypes.c_void_p, ctypes.c_size_t, ctypes.c_void_p)
        self.with_dict.restype = ctypes.c_size_t
        self.context = self.create_ctx(False)
        self.dictionaries = []

    def dictionary(self, data):
        buf = ctypes.create_string_buffer(data)
        ptr = self.create_dict(Span(ctypes.cast(buf, ctypes.c_void_p), len(data)), False)
        if not ptr:
            raise ValueError("Could not create ZSTD dictionary")
        self.dictionaries.append((ptr, buf))
        return ptr

    def decompress(self, data, size=None, dictionary=None, oodle=False):
        if size is None:
            size = frame_size(data)
        if not 0 <= size <= 512 * 1024 * 1024:
            raise ValueError("Invalid decompressed size")
        dst = ctypes.create_string_buffer(size)
        if dictionary is not None:
            actual = self.with_dict(self.context, dst, size, data, len(data), dictionary)
        else:
            actual = (self.oodle if oodle else self.zstd)(dst, size, data, len(data))
        if actual != size:
            raise ValueError(f"Decoded length {actual}, expected {size}")
        return dst.raw


def frame_size(data):
    if data[:4] != b"\x28\xb5\x2f\xfd":
        raise ValueError("ZSTD magic missing")
    flags = data[4]
    single = bool(flags & 0x20)
    code = flags >> 6
    pos = 5 + (0 if single else 1) + (0, 1, 2, 4)[flags & 3]
    length = (1 if single else 0, 2, 4, 8)[code]
    if not length:
        raise ValueError("ZSTD frame has no declared content size")
    size = int.from_bytes(data[pos:pos + length], "little")
    return size + (256 if length == 2 else 0)


def vromfs(path, codec):
    data = Path(path).read_bytes()
    magic = data[:4]
    if magic not in (b"VRFs", b"VRFx"):
        raise ValueError("VROMFS magic missing")
    unpacked, packed = struct.unpack_from("<II", data, 8)
    start = 24 if magic == b"VRFx" else 16
    packed &= 0x3FFFFFF
    body = bytearray(data[start:start + packed])
    pattern = (0xAA55AA55, 0xF00FF00F, 0xAA55AA55, 0x12481248)
    for pos, keys in ((0, pattern), ((len(body) & 0x3FFFFFC) - 16, pattern[::-1])):
        for i, k in enumerate(keys):
            struct.pack_into("<I", body, pos + i * 4,
                             struct.unpack_from("<I", body, pos + i * 4)[0] ^ k)
    image = codec.decompress(bytes(body), unpacked)
    names, count = struct.unpack_from("<II", image)
    records = struct.unpack_from("<I", image, 16)[0]
    result = {}
    for i in range(count):
        at = struct.unpack_from("<Q", image, names + 8 * i)[0]
        name = image[at:image.index(0, at)].decode("utf-8", "surrogateescape")
        off, size = struct.unpack_from("<II", image, records + 16 * i)
        if off + size > len(image):
            raise ValueError("VROMFS entry outside image")
        result[name] = image[off:off + size]
    return result


def vlq(data, pos):
    value = 0
    for shift in range(0, 70, 7):
        c = data[pos]
        pos += 1
        value |= (c & 127) << shift
        if not c & 128:
            return value, pos
    raise ValueError("VLQ too long")


class Blks:
    def __init__(self, files, codec):
        self.codec = codec
        nm = files["\udcff?nm"]
        out = codec.decompress(nm[40:])
        count, pos = vlq(out, 0)
        size, pos = vlq(out, pos)
        self.names = out[pos:pos + size].rstrip(b"\0").decode("utf-8", "surrogateescape").split("\0")
        if len(self.names) != count:
            raise ValueError("Shared name-map size mismatch")
        dictionaries = [data for name, data in files.items() if name.endswith(".dict")]
        if len(dictionaries) != 1:
            raise ValueError(f"Expected one dictionary, got {len(dictionaries)}")
        self.dictionary = codec.dictionary(dictionaries[0])

    def parse(self, data):
        fmt = data[0]
        if fmt == 5:
            buf = self.codec.decompress(data[1:], dictionary=self.dictionary)
        elif fmt == 4:
            buf = self.codec.decompress(data[1:])
        elif fmt == 3:
            buf = data[1:]
        else:
            raise ValueError(f"Unsupported metadata BLK format {fmt}")
        pos = 0
        local_names, pos = vlq(buf, pos)
        if local_names != 0:
            raise ValueError("Unexpected inline names in slim BLK")
        count, pos = vlq(buf, pos)
        nparams, pos = vlq(buf, pos)
        size, pos = vlq(buf, pos)
        pool = buf[pos:pos + size]
        pos += size
        params = []
        for i in range(nparams):
            word, raw = struct.unpack_from("<II", buf, pos + 8 * i)
            name, kind = self.names[word & 0xFFFFFF], word >> 24
            if kind == 1:
                value = self.names[raw & 0x7FFFFFFF] if raw >> 31 else pool[raw:pool.index(0, raw)].decode("utf-8", "surrogateescape")
            elif kind == 2:
                value = struct.unpack("<i", struct.pack("<I", raw))[0]
            elif kind == 3:
                value = struct.unpack("<f", struct.pack("<I", raw))[0]
            elif kind == 9:
                value = bool(raw)
            else:
                value = {"raw_type": kind, "raw_value": raw}
            params.append((name, value))
        pos += nparams * 8
        descs = []
        used = 0
        for i in range(count):
            name, pos = vlq(buf, pos)
            np_, pos = vlq(buf, pos)
            nc, pos = vlq(buf, pos)
            child = 0
            if nc:
                child, pos = vlq(buf, pos)
            descs.append({"name": self.names[name - 1] if name else "root",
                          "params": params[used:used + np_],
                          "child_indices": list(range(child, child + nc))})
            used += np_
        if used != nparams or pos != len(buf):
            raise ValueError("BLK layout not consumed exactly")
        return descs
