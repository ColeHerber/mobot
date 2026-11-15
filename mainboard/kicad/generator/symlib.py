"""Load symbol definitions from KiCad libs / an existing schematic and flatten derived symbols."""
import copy, os
from sexp import parse, find, find1, Sym

STOCK = '/Applications/KiCad/KiCad.app/Contents/SharedSupport/symbols'
CAN_SCH = '/Users/cole/Desktop/Rolly/Esp32 Can+IMU/Esp32 Can+IMU.kicad_sch'

_libcache = {}


def _lib(path):
    if path not in _libcache:
        _libcache[path] = parse(open(path, encoding='utf-8').read())
    return _libcache[path]


def _syms(tree):
    return {x[1]: x for x in tree if isinstance(x, list) and x and x[0] == 'symbol'}


def _flatten(name, syms):
    s = copy.deepcopy(syms[name])
    ext = find1(s, 'extends')
    if not ext:
        return s
    parent = _flatten(ext[1], syms)
    child_props = {p[1]: p for p in find(s, 'property')}
    out = [Sym('symbol'), name]
    for item in parent[2:]:
        if isinstance(item, list) and item and item[0] == 'property':
            out.append(child_props.pop(item[1], item))
        elif isinstance(item, list) and item and item[0] == 'symbol':
            sub = copy.deepcopy(item)
            sub[1] = sub[1].replace(ext[1] + '_', name + '_', 1)
            out.append(sub)
        else:
            out.append(item)
    for p in child_props.values():
        out.append(p)
    return out


def from_stock(lib_id):
    lib, name = lib_id.split(':', 1)
    syms = _syms(_lib(os.path.join(STOCK, lib + '.kicad_sym')))
    return _flatten(name, syms)


def from_file(path, name, lib_id):
    syms = _syms(_lib(path))
    s = _flatten(name, syms)
    return s


def from_can(lib_id):
    tree = _lib(CAN_SCH)
    ls = find1(tree, 'lib_symbols')
    for s in ls[1:]:
        if s[1] == lib_id:
            return copy.deepcopy(s)
    raise KeyError(lib_id)


def embed(sym, lib_id):
    """Return a copy named for embedding in a schematic's lib_symbols."""
    s = copy.deepcopy(sym)
    short = s[1].split(':')[-1]
    s[1] = lib_id
    for sub in find(s, 'symbol'):
        # sub-unit names never carry the library prefix
        base = sub[1].split(':')[-1]
        sub[1] = base if base.startswith(lib_id.split(':')[-1]) else base
    return s


def pins(sym):
    """List of dict(num, name, x, y, ang, unit, etype) for all pins (lib coords, y up)."""
    out = []
    for sub in find(sym, 'symbol'):
        parts = sub[1].rsplit('_', 2)
        unit = int(parts[-2]) if len(parts) == 3 and parts[-2].isdigit() else 0
        style = int(parts[-1]) if parts[-1].isdigit() else 1
        if style not in (0, 1):
            continue
        for p in find(sub, 'pin'):
            at = find1(p, 'at')
            nm = find1(p, 'name')
            num = find1(p, 'number')
            out.append(dict(num=num[1], name=nm[1], x=float(at[1]), y=float(at[2]),
                            ang=float(at[3]) if len(at) > 3 else 0.0, unit=unit,
                            etype=str(p[1])))
    return out
