"""Small glue around two third-party Python packages, installed separately (not part of this tree):
  vromfs   reads the game's .vromfs.bin archives   (modules vromfs.bin, vromfs.vromfs)
  blk      reads binary BLK and writes BLK text    (modules blk.binary, blk.text)
Both are imported as ordinary packages from the Python environment (install them so that `import vromfs` and
`import blk` work); nothing here looks for them in a fixed place.

Compatibility: these packages use the `construct` package and call its stream_read / stream_write / stream_seek /
stream_tell without the trailing `path` argument that newer construct releases require. When the installed construct
requires it, the four functions are wrapped once so that `path` gets a default; with a construct that does not
require it nothing is changed.

  Archive(path).text(inner_path, strict=False) -> BLK text of one file in the archive, or None when it is not there
"""
import inspect
import io


def _construct_compat():
    try:
        import construct as ct
    except ImportError:
        return
    if getattr(ct, "_path_default_added", False):
        return
    try:
        p = inspect.signature(ct.stream_read).parameters.get("path")
    except (TypeError, ValueError, AttributeError):
        return
    if p is None or p.default is not inspect.Parameter.empty:
        return
    _read, _write, _seek, _tell = ct.stream_read, ct.stream_write, ct.stream_seek, ct.stream_tell
    ct.stream_read = lambda stream, length, path="(vromfs)": _read(stream, length, path)
    ct.stream_write = lambda stream, data, length=None, path="(vromfs)": _write(
        stream, data, len(data) if length is None else length, path)
    ct.stream_seek = lambda stream, offset, whence=0, path="(vromfs)": _seek(stream, offset, whence, path)
    ct.stream_tell = lambda stream, path="(vromfs)": _tell(stream, path)
    ct._path_default_added = True


_construct_compat()

try:
    from vromfs.bin import BinFile
    from vromfs.vromfs import VromfsFile
    from blk.binary import BlkType, compose_partial_fat, compose_partial_fat_zst, compose_partial_slim, \
        compose_partial_slim_zst
    import blk.text as blk_text
except ImportError as e:      # a clear message instead of a traceback
    raise SystemExit(f"this script needs the third-party Python packages 'vromfs' and 'blk' (import failed: {e}); "
                     "install them into the Python that runs prep/") from e


class Archive:
    """One .vromfs.bin archive; files looked up by their inner path, case-insensitive."""

    def __init__(self, path):
        self.archive = VromfsFile(BinFile(str(path)))
        self.by_path = {info.path.as_posix().lower(): info for info in self.archive.info_map.values()}

    def paths(self):
        return sorted(self.by_path)

    def value(self, inner):
        info = self.by_path.get(inner.lower())
        if info is None:
            return None
        raw = self.archive.unpack_into(info).getvalue()
        stream, kind = io.BytesIO(raw[1:]), BlkType.from_byte(raw[:1])
        if kind == BlkType.FAT:
            return compose_partial_fat(stream)
        if kind == BlkType.FAT_ZST:
            return compose_partial_fat_zst(stream, self.archive.dctx)
        if kind == BlkType.SLIM:
            return compose_partial_slim(self.archive.nm, stream)
        return compose_partial_slim_zst(self.archive.nm, stream, self.archive.dctx)

    def text(self, inner, strict=False):
        """BLK text of one file: the package's default dialect, or with strict=True its strict dialect."""
        value = self.value(inner)
        if value is None:
            return None
        buf = io.StringIO()
        if strict:
            blk_text.serialize(value, buf, dialect=blk_text.StrictDialect)
        else:
            blk_text.serialize(value, buf)
        return buf.getvalue()
