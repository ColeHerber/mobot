"""Top-level item spans of a KiCad .kicad_sch (text-level, keeps formatting)."""
import re, sys
sys.path.insert(0, __import__('os').path.dirname(__import__('os').path.abspath(__file__)))
from sexp import parse, find, find1


def spans(text):
    """yield (start, end, node) for each direct child of the root list."""
    i = text.index('(') + 1
    depth = 0; start = None; instr = False; esc = False
    for j in range(i, len(text)):
        c = text[j]
        if instr:
            if esc: esc = False
            elif c == '\\': esc = True
            elif c == '"': instr = False
            continue
        if c == '"': instr = True
        elif c == '(':
            if depth == 0: start = j
            depth += 1
        elif c == ')':
            if depth == 0: return
            depth -= 1
            if depth == 0:
                yield start, j + 1, parse(text[start:j + 1])


def f(v): return float(v)


def info(n):
    k = str(n[0]); d = {'kind': k}
    if k == 'symbol':
        d['lib_id'] = find1(n, 'lib_id')[1]
        for p in find(n, 'property'): d[p[1]] = p[2]
        a = find1(n, 'at'); d['at'] = (f(a[1]), f(a[2]), f(a[3]) if len(a) > 3 else 0)
    elif k == 'wire':
        d['pts'] = [(f(p[1]), f(p[2])) for p in find1(n, 'pts')[1:]]
    elif k in ('global_label', 'label', 'text', 'no_connect', 'junction'):
        a = find1(n, 'at'); d['at'] = (f(a[1]), f(a[2])); d['name'] = n[1] if k in ('global_label', 'label', 'text') else ''
    return d
