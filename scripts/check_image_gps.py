"""Refuse any tracked image that carries a GPS position (the owner, 2026-11-14: "add a CI check that refuses any image
with GPS in it"). Ported from Valar-Systems/valar-scopes scripts/check_binary_strings.py (PR #716): the parsing
functions below are copied from it verbatim.

Every TRACKED file (git ls-files) is read. An image is known by its MAGIC, whatever its name says (jpeg, png, webp,
tiff -- the whole file is the TIFF -- and heif/avif, whose 'Exif' item is followed through meta/iinf and meta/iloc),
and also by a jpg/jpeg/png/webp extension. A finding is one of:
  exif-gps      a GPS IFD holding a latitude or longitude tag
  exif-unread   a HEIF whose meta cannot be parsed, or whose Exif item cannot be followed to a TIFF header
  xmp-location  an XMP location property in an image that holds a VALUE: a non-empty attribute value, or an element
                with non-empty text or a non-empty attribute value anywhere inside it. An empty container
                (<rdf:Bag/>, whitespace) holds no location. Two directions are deliberately conservative and are
                KNOWN false positives: an element whose only content is a structural attribute
                (rdf:parseType="Resource") counts, and so does a tag name met where it cannot be judged (inside text)
  container     a zip/3mf/gz/tgz/tar/xz that cannot be read in full (nested, unopenable, or past 64 MB inflated)
Members of a readable container are judged by the same rule, reported as <container>!<member>.

There is NO allowlist and no exemption: an image with GPS is fixed by stripping the GPS, never by excusing the file.

    python3 scripts/check_image_gps.py [--root DIR]   exit 0 clean; 1 a finding; 2 BLIND (git lists no file)
    python3 scripts/check_image_gps.py --selftest     24 rows, synthetic plants in throwaway git repos
    python3 scripts/check_image_gps.py --sabotage     each sabotage, planted in a copy of this script, must turn
                                                      exactly its rows red
It prints paths and kinds only, never a coordinate.
"""
import gzip
import io
import lzma
import os
import re
import shutil
import struct
import subprocess
import sys
import tarfile
import tempfile
import zipfile
import zlib

CAP =64 * 1024 * 1024                          # inflated bytes per container
CONTAINER_EXT = {"zip", "3mf", "gz", "tgz", "tar", "xz"}
KINDS = ("exif-gps", "exif-unread", "xmp-location", "container")

# ---------------------------------------------------------------- copied verbatim from check_binary_strings.py (#716)
PHOTO = {"jpg", "jpeg", "png", "webp"}
XMP_LOC = (b"GPSLatitude", b"GPSLongitude", b"GPSPosition", b"GPSAltitude", b"photoshop:City", b"photoshop:State",
           b"photoshop:Country", b"Iptc4xmpCore:Location", b"LocationShown", b"LocationCreated", b"GpsLatitude",
           b"GpsLongitude")
HEIF_BRANDS = {b"heic", b"heix", b"hevc", b"hevx", b"heim", b"heis", b"mif1", b"msf1", b"avif", b"avis"}


def ext_of(name):
    base = name.replace("\\", "/").rsplit("/", 1)[-1]
    return base.rsplit(".", 1)[-1].lower() if "." in base else ""


def tiff_gps(t):
    """True when a TIFF/EXIF block's IFD0 points at a GPS IFD holding a latitude or longitude tag (1-4)."""
    if len(t) < 8 or t[:2] not in (b"II", b"MM"):
        return False
    e = "<" if t[:2] == b"II" else ">"
    try:
        off = struct.unpack(e + "I", t[4:8])[0]
        for i in range(struct.unpack(e + "H", t[off:off + 2])[0]):
            tag, _, _, val = struct.unpack(e + "HHII", t[off + 2 + 12 * i:off + 14 + 12 * i])
            if tag == 0x8825:
                tags = {struct.unpack(e + "H", t[val + 2 + 12 * j:val + 4 + 12 * j])[0]
                        for j in range(struct.unpack(e + "H", t[val:val + 2])[0])}
                return bool(tags & {1, 2, 3, 4})
    except (struct.error, IndexError):
        return False
    return False


def exif_blocks(data, ext):
    """The EXIF (TIFF) blocks of a JPEG's APP1 segments, a PNG's eXIf chunks or a WebP's EXIF chunk."""
    out = []
    try:
        if ext in ("jpg", "jpeg") and data[:2] == b"\xff\xd8":
            i = 2
            while i + 4 <= len(data) and data[i] == 0xFF and data[i + 1] not in (0xD9, 0xDA):
                ln = struct.unpack(">H", data[i + 2:i + 4])[0]
                seg = data[i + 4:i + 2 + ln]
                if data[i + 1] == 0xE1 and seg[:6] == b"Exif\x00\x00":
                    out.append(seg[6:])
                i += 2 + ln
        elif ext == "png" and data[:8] == b"\x89PNG\r\n\x1a\n":
            i = 8
            while i + 8 <= len(data):
                ln = struct.unpack(">I", data[i:i + 4])[0]
                if data[i + 4:i + 8] == b"eXIf":
                    out.append(data[i + 8:i + 8 + ln])
                i += 12 + ln
        elif ext == "webp" and data[:4] == b"RIFF":
            i = 12
            while i + 8 <= len(data):
                ln = struct.unpack("<I", data[i + 4:i + 8])[0]
                if data[i:i + 4] == b"EXIF":
                    blk = data[i + 8:i + 8 + ln]
                    out.append(blk[6:] if blk[:6] == b"Exif\x00\x00" else blk)
                i += 8 + ln + (ln & 1)
    except struct.error:
        pass
    return out


def image_kind(raw):
    """An image kind by MAGIC, whatever the name says: jpeg, png, webp, tiff or heif (heif and avif brands); else None."""
    if raw[:3] == b"\xff\xd8\xff":
        return "jpeg"
    if raw[:8] == b"\x89PNG\r\n\x1a\n":
        return "png"
    if raw[:4] == b"RIFF" and raw[8:12] == b"WEBP":
        return "webp"
    if raw[:4] in (b"II*\x00", b"MM\x00*"):
        return "tiff"
    if raw[4:8] == b"ftyp" and len(raw) >= 16:
        n = struct.unpack(">I", raw[:4])[0]
        brands = {raw[8:12]} | {raw[i:i + 4] for i in range(16, min(n, len(raw)) - 3, 4)}
        if brands & HEIF_BRANDS:
            return "heif"
    return None


def _boxes(d, start, end):
    """(type, payload start, box end) for each ISO-BMFF box in d[start:end]; ValueError on a size that does not fit."""
    i = start
    while i + 8 <= end:
        n, t, h = struct.unpack(">I", d[i:i + 4])[0], d[i + 4:i + 8], 8
        if n == 1:
            n, h = struct.unpack(">Q", d[i + 8:i + 16])[0], 16
        elif n == 0:
            n = end - i
        if n < h or i + n > end:
            raise ValueError("box size")
        yield t, i + h, i + n
        i += n


def heif_exif(d):
    """([TIFF block], unread) of a HEIF: each item of type 'Exif' (meta/iinf), followed through meta/iloc (construction
    method 0, every extent inside the file) to its TIFF header, at 4 + the item's leading big-endian offset. unread is
    True when the meta cannot be parsed or an Exif item cannot be followed: blind is never reported as absent."""
    try:
        meta = next(((p, e) for t, p, e in _boxes(d, 0, len(d)) if t == b"meta"), None)
        if meta is None:
            return [], False
        kids = {t: (p, e) for t, p, e in _boxes(d, meta[0] + 4, meta[1])}       # meta is a full box
        ids = set()
        if b"iinf" in kids:
            p, e = kids[b"iinf"]
            for t, ip, _ in _boxes(d, p + 4 + (2 if d[p] == 0 else 4), e):
                if t != b"infe" or d[ip] < 2:
                    continue
                w = 2 if d[ip] == 2 else 4
                if d[ip + 4 + w + 2:ip + 4 + w + 6] == b"Exif":
                    ids.add(int.from_bytes(d[ip + 4:ip + 4 + w], "big"))
        if not ids:
            return [], False
        locs = {}
        if b"iloc" in kids:
            p, _ = kids[b"iloc"]
            v, q = d[p], p + 4
            osz, lsz, bsz, isz = d[q] >> 4, d[q] & 15, d[q + 1] >> 4, (d[q + 1] & 15 if v in (1, 2) else 0)
            q += 2

            def rd(width):
                nonlocal q
                x = int.from_bytes(d[q:q + width], "big")
                q += width
                return x
            for _ in range(rd(4 if v == 2 else 2)):
                iid = rd(4 if v == 2 else 2)
                cm = rd(2) & 15 if v in (1, 2) else 0
                rd(2)                                                            # data_reference_index
                base, ext = rd(bsz), []
                for _ in range(rd(2)):
                    if isz:
                        rd(isz)
                    off = rd(osz)
                    ext.append((base + off, rd(lsz)))
                locs[iid] = (cm, ext)
        blocks, unread = [], False
        for iid in sorted(ids):
            cm, ext = locs.get(iid, (None, []))
            if cm != 0 or not ext or any(n <= 0 or o + n > len(d) for o, n in ext):
                unread = True
                continue
            item = b"".join(d[o:o + n] for o, n in ext)
            tiff = item[4 + struct.unpack(">I", item[:4])[0]:] if len(item) >= 4 else b""
            if tiff[:4] in (b"II*\x00", b"MM\x00*"):
                blocks.append(tiff)
            else:
                unread = True
        return blocks, unread
    except (ValueError, struct.error, IndexError):
        return [], True


def exif_magic(raw, kind):
    """(EXIF blocks, unread) by the MAGIC kind; ([], False) for a file that is not an image."""
    if kind in ("jpeg", "png", "webp"):
        return exif_blocks(raw, {"jpeg": "jpg", "png": "png", "webp": "webp"}[kind]), False
    if kind == "tiff":
        return [raw], False
    if kind == "heif":
        return heif_exif(raw)
    return [], False


_ATTRS = re.compile(rb"""[\w:.-]+\s*=\s*("[^"]*"|'[^']*')""")


def _attrs_have_value(s):
    """True when any attribute in s has a non-empty value."""
    return any(m.group(1)[1:-1].strip() for m in _ATTRS.finditer(s))


def xmp_location_value(raw):
    """True when an XMP location property (XMP_LOC) holds a VALUE. Every occurrence of a token is accounted for:
    an attribute holds a value when it is non-empty; a tag when its own attributes hold one (a structural
    rdf:parseType counts: the conservative direction), or when its content up to the matching close holds non-empty
    text or a non-empty attribute value at any depth; a closing tag holds none; and an occurrence that is neither an
    attribute nor a tag cannot be judged, so it counts. Fires only where a token occurs: a subset of token presence."""
    for tok in XMP_LOC:
        t = re.escape(tok)
        seen = 0
        for m in re.finditer(rb"(?<![A-Za-z])" + t + rb"""\s*=\s*("[^"]*"|'[^']*')""", raw):
            seen += 1
            if m.group(1)[1:-1].strip():
                return True
        for m in re.finditer(rb"<(/?)(?:[\w.-]+:)?" + t + rb"""\b((?:[^>"']|"[^"]*"|'[^']*')*)>""", raw):
            seen += 1
            if m.group(1):
                continue                                              # a closing tag
            attrs = m.group(2)
            if _attrs_have_value(attrs):
                return True                                           # its own attributes, structural ones included
            if attrs.rstrip().endswith(b"/"):
                continue                                              # self-closing, and nothing in it
            end = re.search(rb"</(?:[\w.-]+:)?" + t + rb"\s*>", raw[m.end():])
            if end is None:
                return True                                           # unclosed: cannot be judged
            inner = raw[m.end():m.end() + end.start()]
            if re.sub(rb"<[^>]*>", b"", inner).strip():
                return True                                           # text
            if _attrs_have_value(inner):
                return True                                           # a value nested at any depth
        if raw.count(tok) > seen:
            return True                                               # an occurrence that is neither
    return False


def _tiff_gps():
    """A minimal little-endian TIFF whose IFD0 points at a GPS IFD holding a latitude (tag 2)."""
    t = bytearray(b"II*\x00" + struct.pack("<I", 8))
    t += struct.pack("<H", 1) + struct.pack("<HHII", 0x8825, 4, 1, 26) + struct.pack("<I", 0)   # IFD0 at 8
    t += struct.pack("<H", 1) + struct.pack("<HHII", 2, 5, 3, 44) + struct.pack("<I", 0)        # GPS IFD at 26
    t += struct.pack("<6I", 1, 1, 2, 1, 3, 1)                                                     # 3 rationals at 44
    return bytes(t)


def _tiff_plain():
    """A minimal little-endian TIFF with NO GPS IFD: IFD0 holds one Orientation entry."""
    return (b"II*\x00" + struct.pack("<I", 8) + struct.pack("<H", 1) + struct.pack("<HHIHH", 0x0112, 3, 1, 1, 0)
            + struct.pack("<I", 0))


def _box(t, payload):
    return struct.pack(">I", 8 + len(payload)) + t + payload


def _heif(tiff, with_iloc=True):
    """A minimal HEIF: ftyp heic; meta holding iinf (one 'Exif' infe) and, unless with_iloc is False, iloc pointing into
    mdat; mdat holding the Exif item (a 4-byte offset of 6, "Exif" and two NULs, then the TIFF)."""
    item = struct.pack(">I", 6) + b"Exif\x00\x00" + tiff
    ftyp = _box(b"ftyp", b"heic" + struct.pack(">I", 0) + b"mif1heic")
    infe = _box(b"infe", bytes([2, 0, 0, 0]) + struct.pack(">HH", 1, 0) + b"Exif" + b"\x00")
    iinf = _box(b"iinf", bytes([0, 0, 0, 0]) + struct.pack(">H", 1) + infe)

    def meta(off):
        iloc = _box(b"iloc", bytes([0, 0, 0, 0, 0x44, 0x00]) + struct.pack(">HHHHII", 1, 1, 0, 1, off, len(item)))
        return _box(b"meta", bytes([0, 0, 0, 0]) + iinf + (iloc if with_iloc else b""))
    off = len(ftyp) + len(meta(0)) + 8
    return ftyp + meta(off) + _box(b"mdat", item)


# ------------------------------------------------------------------------------------------------- containers

class Unopenable(Exception):
    """A container this cannot read in full: it fails as `container`."""


def _capped(stream, budget):
    data = stream.read(budget + 1)
    if len(data) > budget:
        raise Unopenable("past the cap")
    return data


def _nested(name, data):
    return ext_of(name) in CONTAINER_EXT or data[:4] == b"PK\x03\x04" or data[:2] == b"\x1f\x8b" or data[:6] == b"\xfd7zXZ\x00"


def members(ext, raw):
    """[(name, bytes)] of a container's members; Unopenable if it cannot be read in full."""
    try:
        if ext in ("zip", "3mf"):
            out, used = [], 0
            with zipfile.ZipFile(io.BytesIO(raw)) as z:
                for info in z.infolist():
                    if info.is_dir():
                        continue
                    with z.open(info) as fh:
                        data = _capped(fh, CAP - used)
                    used += len(data)
                    out.append((info.filename, data))
        elif ext in ("gz", "tgz", "xz"):
            fh = gzip.GzipFile(fileobj=io.BytesIO(raw)) if ext != "xz" else lzma.LZMAFile(io.BytesIO(raw))
            with fh:
                data = _capped(fh, CAP)
            out = _tar(data) if data[257:262] == b"ustar" else [("member-of-." + ext, data)]
        else:
            out = _tar(raw)
    except (zipfile.BadZipFile, OSError, EOFError, lzma.LZMAError, tarfile.TarError, zlib.error) as e:
        raise Unopenable(type(e).__name__)
    for name, data in out:
        if _nested(name, data):
            raise Unopenable("a nested container")
    return out


def _tar(blob):
    out, used = [], 0
    with tarfile.open(fileobj=io.BytesIO(blob)) as t:
        for m in t.getmembers():
            if not m.isfile():
                continue
            data = _capped(t.extractfile(m), CAP - used)
            used += len(data)
            out.append((m.name, data))
    return out


# ------------------------------------------------------------------------------------------------- the scan

def kinds_of_bytes(ext, raw):
    """The findings in one file's (or one member's) bytes; the same lines as check_binary_strings.py's photo rule."""
    found = set()
    # An image by its MAGIC as well as by its name: the extension path is kept whole and the magic path adds to it.
    kind = image_kind(raw)
    blocks, unread = exif_magic(raw, kind)
    if ext in PHOTO:
        blocks = exif_blocks(raw, ext) + blocks
    if any(tiff_gps(b) for b in blocks):
        found.add("exif-gps")
    if unread:
        found.add("exif-unread")
    if (ext in PHOTO or kind) and b"x:xmpmeta" in raw and xmp_location_value(raw):
        found.add("xmp-location")
    return found


def tracked(root):
    p = subprocess.run(["git", "-C", root, "ls-files", "-z"], capture_output=True)
    if p.returncode != 0:
        return None
    return [x for x in p.stdout.decode("utf-8", "replace").split("\0") if x]


def judge(root, files):
    """(files read, [(path, kind)]). Never a value."""
    read, findings = 0, []
    for rel in files:
        p = os.path.join(root, *rel.split("/"))
        if os.path.isdir(p):                     # a submodule's gitlink: its own repo is checked on its own
            continue
        raw = open(p, "rb").read()
        read += 1
        ext = ext_of(rel)
        if ext in CONTAINER_EXT:
            try:
                ms = members(ext, raw)
            except Unopenable:
                findings.append((rel, "container"))
                continue
            findings += [(rel + "!" + name, k) for name, data in ms for k in kinds_of_bytes(ext_of(name), data)]
        else:
            findings += [(rel, k) for k in kinds_of_bytes(ext, raw)]
    return read, sorted(findings)


def check(root, out=print):
    files = tracked(root)
    if not files:
        out("image gps: BLIND -- git lists no tracked file under %s" % root)
        return 2, []
    read, findings = judge(root, files)
    out("image gps: %d tracked file(s), %d read; findings %d" % (len(files), read, len(findings)))
    for rel, kind in findings:
        out("  FINDING %s: %s" % (kind, rel))
    out("image gps: " + ("FAILED -- strip the GPS from the file; there is no allowlist" if findings else "OK"))
    return (1 if findings else 0), findings


# ------------------------------------------------------------------------------------------------- selftest

def _jpeg(*segments, tail=b""):
    app0 = b"\xff\xe0" + struct.pack(">H", 16) + b"JFIF\x00\x01\x01\x00\x00\x01\x00\x01\x00\x00"
    return b"\xff\xd8" + app0 + b"".join(segments) + b"\xff\xd9" + tail


def _exif_segment(tiff):
    body = b"Exif\x00\x00" + tiff
    return b"\xff\xe1" + struct.pack(">H", len(body) + 2) + body


def _png(tiff=None):
    ihdr = struct.pack(">I", 13) + b"IHDR" + struct.pack(">IIBBBBB", 1, 1, 8, 2, 0, 0, 0) + b"\x00" * 4
    exif = (struct.pack(">I", len(tiff)) + b"eXIf" + tiff + b"\x00" * 4) if tiff else b""
    return b"\x89PNG\r\n\x1a\n" + ihdr + exif + b"\x00\x00\x00\x00IEND\xaeB`\x82"


def _webp(tiff):
    blk = b"Exif\x00\x00" + tiff
    chunk = b"EXIF" + struct.pack("<I", len(blk)) + blk + (b"\x00" if len(blk) & 1 else b"")
    return b"RIFF" + struct.pack("<I", 4 + len(chunk)) + b"WEBP" + chunk


def _xmp(body):
    """An XMP packet holding body inside rdf:RDF."""
    return b'<x:xmpmeta xmlns:x="adobe:ns:meta/"><rdf:RDF>' + body + b"</rdf:RDF></x:xmpmeta>"


def _zip(name, data):
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w") as z:
        z.writestr(name, data, compress_type=zipfile.ZIP_DEFLATED)
    return buf.getvalue()


def selftest():
    gps, plain = _tiff_gps(), _tiff_plain()
    xmp = b'<x:xmpmeta xmlns:x="adobe:ns:meta/"><rdf:Description exif:GPSLatitude="1,0N"/></x:xmpmeta>'
    # (number, label, {tracked path: bytes}, {untracked path: bytes}, want: [(path, kind)] or "BLIND")
    rows = [
        (1, "control: a JPEG with no APP1", {"a.jpg": _jpeg()}, {}, []),
        (2, "a JPEG with an EXIF GPS IFD", {"a.jpg": _jpeg(_exif_segment(gps))}, {}, [("a.jpg", "exif-gps")]),
        (3, "control: a JPEG whose EXIF has no GPS IFD", {"a.jpg": _jpeg(_exif_segment(plain))}, {}, []),
        (4, "a PNG with an eXIf GPS chunk", {"a.png": _png(gps)}, {}, [("a.png", "exif-gps")]),
        (5, "a WebP with an EXIF GPS chunk", {"a.webp": _webp(gps)}, {}, [("a.webp", "exif-gps")]),
        (6, "a GPS TIFF, .tif", {"scan.tif": gps}, {}, [("scan.tif", "exif-gps")]),
        (7, "control: a TIFF with no GPS IFD", {"scan.tif": plain}, {}, []),
        (8, "a GPS HEIF, .heic", {"p.heic": _heif(gps)}, {}, [("p.heic", "exif-gps")]),
        (9, "a HEIF whose Exif item has no iloc", {"p.heic": _heif(gps, False)}, {}, [("p.heic", "exif-unread")]),
        (10, "control: a HEIF with no GPS", {"p.heic": _heif(plain)}, {}, []),
        (11, "a GPS JPEG named .bin", {"blob.bin": _jpeg(_exif_segment(gps))}, {}, [("blob.bin", "exif-gps")]),
        (12, "a GPS PNG named .jpg", {"thumb.jpg": _png(gps)}, {}, [("thumb.jpg", "exif-gps")]),
        (13, "a JPEG with an XMP GPSLatitude", {"a.jpg": _jpeg(tail=xmp)}, {}, [("a.jpg", "xmp-location")]),
        (14, "a GPS JPEG member of a .zip", {"pack.zip": _zip("img/a.jpg", _jpeg(_exif_segment(gps)))}, {},
         [("pack.zip!img/a.jpg", "exif-gps")]),
        (15, "an unopenable .zip", {"pack.zip": b"PK\x03\x04" + b"\x00" * 26}, {}, [("pack.zip", "container")]),
        (16, "control: a GPS JPEG present but not tracked", {"a.jpg": _jpeg()},
         {"loose.jpg": _jpeg(_exif_segment(gps))}, []),
        (17, "an empty repo is BLIND", {}, {}, "BLIND"),
        # An XMP location property counts only when it holds a VALUE (valar-scopes #717)
        (18, "control: an empty LocationCreated Bag",
         {"a.jpg": _jpeg(tail=_xmp(b"<rdf:Description><Iptc4xmpExt:LocationCreated><rdf:Bag/>"
                                   b"</Iptc4xmpExt:LocationCreated></rdf:Description>"))}, {}, []),
        (19, "control: whitespace-only LocationShown",
         {"a.jpg": _jpeg(tail=_xmp(b"<rdf:Description><Iptc4xmpExt:LocationShown>  \n  "
                                   b"</Iptc4xmpExt:LocationShown></rdf:Description>"))}, {}, []),
        (20, "a parseType-only LocationShown counts",
         {"a.jpg": _jpeg(tail=_xmp(b"<rdf:Description><Iptc4xmpExt:LocationShown rdf:parseType=\"Resource\">"
                                   b"</Iptc4xmpExt:LocationShown></rdf:Description>"))}, {}, [("a.jpg", "xmp-location")]),
        (21, "a filled photoshop:City attribute",
         {"a.jpg": _jpeg(tail=_xmp(b"<rdf:Description photoshop:City=\"Somewhere\"/>"))}, {},
         [("a.jpg", "xmp-location")]),
        (22, "a City attribute nested in a Bag",
         {"a.jpg": _jpeg(tail=_xmp(b"<rdf:Description><Iptc4xmpExt:LocationCreated><rdf:Bag><rdf:li "
                                   b"Iptc4xmpExt:City=\"Somewhere\"/></rdf:Bag></Iptc4xmpExt:LocationCreated>"
                                   b"</rdf:Description>"))}, {}, [("a.jpg", "xmp-location")]),
        (23, "GPSLatitude as element text",
         {"a.jpg": _jpeg(tail=_xmp(b"<rdf:Description><exif:GPSLatitude>12,34.5N</exif:GPSLatitude>"
                                   b"</rdf:Description>"))}, {}, [("a.jpg", "xmp-location")]),
        (24, "a location name inside text counts",
         {"a.jpg": _jpeg(tail=_xmp(b"<rdf:Description><dc:description><rdf:Alt><rdf:li xml:lang=\"x-default\">"
                                   b"near the LocationShown spot</rdf:li></rdf:Alt></dc:description>"
                                   b"</rdf:Description>"))}, {}, [("a.jpg", "xmp-location")]),
    ]
    g = ["-c", "user.name=selftest", "-c", "user.email=selftest@example.invalid", "-c", "core.autocrlf=false"]
    ok = True
    for num, label, tracked_files, loose, want in rows:
        d = tempfile.mkdtemp(prefix="imggps-")
        try:
            subprocess.run(["git", "-C", d] + g + ["init", "-q"], check=True)
            for rel, data in list(tracked_files.items()) + list(loose.items()):
                p = os.path.join(d, *rel.split("/"))
                os.makedirs(os.path.dirname(p), exist_ok=True)
                open(p, "wb").write(data)
            if tracked_files:
                subprocess.run(["git", "-C", d] + g + ["add", "--"] + sorted(tracked_files), check=True)
            rc, findings = check(d, out=lambda s: None)
        finally:
            shutil.rmtree(d, ignore_errors=True)
        good = (rc == 2) if want == "BLIND" else (rc == (1 if want else 0) and findings == sorted(want))
        ok &= good
        print("  %s %2d %-46s exit %d findings %s" % ("ok  " if good else "FAIL", num, label, rc,
                                                    [k for _, k in findings] or "-"))
    print("selftest " + ("PASSED" if ok else "FAILED") + ": %d rows" % len(rows))
    return 0 if ok else 1


# ------------------------------------------------------------------------------------------------- sabotage

# Each anchor occurs exactly once in this file: the constants are split so that this table holds no second copy.
SABOTAGES = {
    "S-MAGIC": ("    kind = image_" + "kind(raw)\n", "    kind = None  # SABOTAGE S-MAGIC: by extension only\n",
                {6, 8, 9, 11, 12}),
    "S-TIFF": ("    if kind == \"ti" + "ff\":\n        return [raw], False\n",
               "    if kind == \"tiff\":  # SABOTAGE S-TIFF\n        return [], False\n", {6}),
    "S-HEIF": ("    if kind == \"he" + "if\":\n        return heif_exif(raw)\n",
               "    if kind == \"heif\":  # SABOTAGE S-HEIF\n        return [], False\n", {8, 9}),
    "S-XMP": ("    if (ext in PHOTO or kind) and b\"x:xmp" + "meta\" in raw",
              "    if False and b\"x:xmpmeta\" in raw", {13, 20, 21, 22, 23, 24}),
    "S-XMPVAL": ("in raw and xmp_location_" + "value(raw):\n",
                 "in raw and any(t in raw for t in XMP_LOC):  # SABOTAGE S-XMPVAL: token presence again\n", {18, 19}),
    "S-XMPATTR": ("            if m.group(1)[1:-1]" + ".strip():\n                return True\n",
                  "            if False:  # SABOTAGE S-XMPATTR\n                return True\n", {13, 21}),
    "S-XMPOPEN": ("            if _attrs_have_value(at" + "trs):\n", "            if False:  # SABOTAGE S-XMPOPEN\n", {20}),
    "S-XMPTEXT": ("            if re.sub(rb\"<[^>]*>\", b\"\", in" + "ner).strip():\n",
                  "            if False:  # SABOTAGE S-XMPTEXT\n", {23}),
    "S-XMPNEST": ("            if _attrs_have_value(in" + "ner):\n", "            if False:  # SABOTAGE S-XMPNEST\n", {22}),
    "S-XMPUNACC": ("        if raw.count(tok) > se" + "en:\n", "        if False:  # SABOTAGE S-XMPUNACC\n", {24}),
    "S-CONTAINER": ("        if ext in CONTAINER_" + "EXT:\n            try:\n",
                    "        if False:  # SABOTAGE S-CONTAINER\n            try:\n", {14, 15}),
}
ROW = re.compile(r"^\s*(ok|FAIL)\s+(\d+)\s")
NROWS = 24


def _selftest_of(src):
    with tempfile.TemporaryDirectory() as td:
        p = os.path.join(td, "check_image_gps.py")
        open(p, "w", encoding="utf-8", newline="\n").write(src)
        r = subprocess.run([sys.executable, "-I", p, "--selftest"], capture_output=True, text=True)
    marks = [m for m in (ROW.match(l) for l in r.stdout.splitlines()) if m]
    return r.returncode, {int(m.group(2)) for m in marks if m.group(1) == "FAIL"}, len(marks)


def sabotage():
    src = open(os.path.abspath(__file__), encoding="utf-8").read().replace("\r\n", "\n")
    rc, red, n = _selftest_of(src)
    ok = rc == 0 and not red and n == NROWS
    print("  %s control: the unplanted copy: exit %d, %d rows, red %s" % ("ok  " if ok else "FAIL", rc, n,
                                                                       sorted(red) or "-"))
    for name, (anchor, repl, want) in SABOTAGES.items():
        if src.count(anchor) != 1:
            print("  FAIL %s: its anchor occurs %d times, need exactly 1 -- not planted" % (name, src.count(anchor)))
            ok = False
            continue
        planted = src.replace(anchor, repl)
        assert planted != src
        rc, red, n = _selftest_of(planted)
        good = rc == 1 and red == want and n == NROWS
        ok &= good
        print("  %s %s: exit %d, red %s (want %s)" % ("ok  " if good else "FAIL", name, rc, sorted(red), sorted(want)))
    print("SABOTAGE " + ("PASSED" if ok else "FAILED"))
    return 0 if ok else 1


def main(argv):
    if "--selftest" in argv:
        return selftest()
    if "--sabotage" in argv:
        return sabotage()
    if "--root" in argv:
        root = argv[argv.index("--root") + 1]
    else:                                        # the repo this script sits in, wherever in it the script is
        p = subprocess.run(["git", "-C", os.path.dirname(os.path.abspath(__file__)), "rev-parse", "--show-toplevel"],
                           capture_output=True, text=True)
        root = p.stdout.strip() if p.returncode == 0 else ""
    return check(root)[0]


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
