"""Single-sheet schematic writer.

Every symbol pin gets a global label, a power symbol, or a no-connect flag (CAN+IMU board style).
All text/body extents are estimated and checked for overlaps (Sheet.overlaps()).
"""
import uuid, math
from sexp import Sym, dump, find, find1
import symlib

PROJECT = 'mobot_mainboard'
VERSION = '20260306'
G = 1.27
CH = 1.0          # estimated stroke-font advance per character at 1.27 mm
POWER = {'GND': 'power:GND', '+3.3V': 'power:+3.3V', '+5V': 'power:+5V'}
S = Sym


def uid():
    return str(uuid.uuid4())


def snap(v, g=G):
    return round(round(v / g) * g, 4)


def effects(justify=None, size=1.27):
    e = [S('effects'), [S('font'), [S('size'), size, size]]]
    if justify:
        e.append([S('justify')] + [S(j) for j in justify.split()])
    return e


def prop(name, value, x, y, rot=0, hide=False, justify=None, size=1.27):
    p = [S('property'), name, value, [S('at'), x, y, rot]]
    if hide:
        p.append([S('hide'), S('yes')])
    p += [[S('show_name'), S('no')], [S('do_not_autoplace'), S('no')], effects(justify, size=size)]
    return p


def rotvec(x, y, r):
    """rotate a screen vector (y down) CCW by r degrees."""
    r %= 360
    return {0: (x, y), 90: (y, -x), 180: (-x, -y), 270: (-y, x)}[r]


def dir_to_rot(dx, dy):
    if abs(dx) > abs(dy):
        return 0 if dx > 0 else 180
    return 270 if dy > 0 else 90


def tw(s, size=1.27):
    return len(s) * CH * size / 1.27


class Sheet:
    def __init__(self, paper=(1189, 841)):
        self.paper = paper
        self.uuid = uid()
        self.items = []
        self.libs = {}
        self.syms = []
        self.nets_used = {}
        self.ox = self.oy = 0.0
        self.boxes = []      # (kind, desc, x0, y0, x1, y1, owner)
        self.cur_box = None
        self.box_of = []
        self.segs = []       # absolute wire segments
        self.pin_pts = set() # absolute pin end points (all placed pins)
        self.anchor_pts = set()
        self._own = 0

    # ---------------------------------------------------------------- geometry bookkeeping
    def region(self, x, y):
        self.ox, self.oy = snap(x, 2.54), snap(y, 2.54)

    def _bb(self, kind, desc, x0, y0, x1, y1, owner):
        self.boxes.append((kind, desc, min(x0, x1), min(y0, y1), max(x0, x1), max(y0, y1), owner))
        self.box_of.append(self.cur_box)

    def _txt_bb(self, s, x, y, rot, justify, owner, desc, size=1.27, kind='text'):
        """bbox of a text anchored at (x,y); rot 0/90 only (KiCad keeps text readable)."""
        w, h = tw(s, size), size * 1.15
        j = justify or 'center'
        if rot % 180 == 0:
            x0 = x if 'left' in j else (x - w if 'right' in j else x - w / 2)
            y0 = y - h if 'bottom' in j else (y if 'top' in j else y - h / 2)
            self._bb(kind, desc, x0, y0, x0 + w, y0 + h, owner)
        else:
            y1 = y if 'left' in j else (y + w if 'right' in j else y + w / 2)
            self._bb(kind, desc, x - h / 2, y1 - w, x + h / 2, y1, owner)

    def lib(self, lib_id, source):
        if lib_id not in self.libs:
            self.libs[lib_id] = symlib.embed(source, lib_id)
        return self.libs[lib_id]

    # ---------------------------------------------------------------- primitives (absolute coords)
    def _text(self, txt, x, y, size=1.27, owner=None):
        self.items.append([S('text'), txt, [S('exclude_from_sim'), S('no')], [S('at'), x, y, 0],
                           effects('left bottom', size=size), [S('uuid'), uid()]])
        self._txt_bb(txt, x, y, 0, 'left bottom', owner, 'note:' + txt[:20], size)

    def _label(self, net, x, y, rot, shape, owner):
        j = {0: 'left', 180: 'right', 90: 'left', 270: 'right'}[rot]
        self.items.append([S('global_label'), net, [S('shape'), S(shape)], [S('at'), x, y, rot],
                           [S('fields_autoplaced'), S('yes')], effects(j), [S('uuid'), uid()],
                           [S('property'), 'Intersheetrefs', '${INTERSHEET_REFS}', [S('at'), x, y, 0],
                            [S('hide'), S('yes')], [S('show_name'), S('no')], [S('do_not_autoplace'), S('no')],
                            effects()]])
        L = tw(net) + 2.6
        dx, dy = rotvec(1, 0, rot) if rot in (0, 180) else ((0, -1) if rot == 90 else (0, 1))
        if rot in (0, 180):
            self._bb('label', 'label:' + net, x + 0.3 * dx, y - 1.0, x + L * dx, y + 1.0, owner)
        else:
            self._bb('label', 'label:' + net, x - 1.0, y + 0.3 * dy, x + 1.0, y + L * dy, owner)
        self.nets_used[net] = self.nets_used.get(net, 0) + 1

    def _noconn(self, x, y):
        self.items.append([S('no_connect'), [S('at'), x, y], [S('uuid'), uid()]])

    def _power(self, net, x, y, out=(0, 1), flag=False, owner=None):
        lib_id = 'power:PWR_FLAG' if flag else POWER[net]
        src = symlib.from_stock(lib_id) if flag else symlib.from_can(lib_id)
        self.lib(lib_id, src)
        base = (0, -1) if (flag or net != 'GND') else (0, 1)
        rot = 0
        for r in (0, 90, 180, 270):
            if rotvec(base[0], base[1], r) == out:
                rot = r
        if flag:
            rot, out = 0, (0, -1)
        label = 'PWR_FLAG' if flag else net
        w = tw(label)
        if out[0] != 0:   # sideways symbol: text beyond the symbol body
            tx, ty = x + out[0] * (3.6 + w / 2), y
        else:
            tx, ty = x, y + out[1] * 5.6
        node = [S('symbol'), [S('lib_id'), lib_id], [S('at'), x, y, rot], [S('unit'), 1], [S('body_style'), 1],
                [S('exclude_from_sim'), S('no')], [S('in_bom'), S('yes')], [S('on_board'), S('yes')],
                [S('in_pos_files'), S('yes')], [S('dnp'), S('no')], [S('uuid'), uid()],
                prop('Reference', ('#FLG' if flag else '#PWR') + '?', tx, ty, hide=True),
                prop('Value', label, tx, ty, rot=rot % 180, hide=flag),
                prop('Footprint', '', x, y, hide=True), prop('Datasheet', '', x, y, hide=True),
                [S('pin'), '1', [S('uuid'), uid()]]]
        self.items.append(node)
        self.syms.append((node, None))
        # body (arrow / ground bars) + text
        px, py = -out[1], out[0]
        self._bb('power', 'pwr:' + label, x + out[0] * 0.4 - 1.4 * abs(px), y + out[1] * 0.4 - 1.4 * abs(py),
                 x + out[0] * 3.3 + 1.4 * abs(px), y + out[1] * 3.3 + 1.4 * abs(py), owner)
        if not flag:
            self._txt_bb(label, tx, ty, 0, None, owner, 'pwrtxt:' + label)
            self.nets_used[net] = self.nets_used.get(net, 0) + 1

    # ---------------------------------------------------------------- public (region-relative coords)
    def text(self, txt, x, y, size=1.27):
        self._text(txt, self.ox + x, self.oy + y, size)

    def note(self, x, y, lines):
        for i, ln in enumerate(lines):
            self.text(ln, x, y + i * 2.54)

    def box(self, x, y, w, h, title):
        X, Y = self.ox + x, self.oy + y
        self.cur_box = (title, X, Y, X + w, Y + h)
        self.items.append([S('rectangle'), [S('start'), X, Y], [S('end'), X + w, Y + h],
                           [S('stroke'), [S('width'), 0], [S('type'), S('default')]],
                           [S('fill'), [S('type'), S('none')]], [S('uuid'), uid()]])
        self._text(title, X + 1.27, Y + 2.54)

    def flag(self, net, x, y):
        """PWR_FLAG on a net: flag above the point, label/power symbol below."""
        X, Y = snap(self.ox + x, 2.54), snap(self.oy + y, 2.54)
        self._own += 1
        self._power(net, X, Y, flag=True, owner=('flag', self._own))
        if net in POWER:
            self._power(net, X, Y, out=(0, 1), owner=('flag', self._own))
        else:
            self._label(net, X, Y, 270, 'passive', ('flag', self._own))

    def place(self, lib_id, ref, value, x, y, nets, rot=0, footprint='', src=None, dnp=False,
              shapes=None, extra=None, fields=None, datasheet=''):
        """nets: {pin_number: net | None(no-connect)}.  fields: 'above'|'below'|'right'|'left'|(dx,dy) for ref."""
        x, y = snap(self.ox + x, 1.27), snap(self.oy + y, 1.27)
        if src is None:
            src = symlib.from_stock(lib_id)
        sdef = self.lib(lib_id, src)
        pinmap = {}
        pins = symlib.pins(sdef)
        shapes = shapes or {}
        self._own += 1
        owner = ('sym', self._own)
        groups = {}
        for p in pins:
            if p['unit'] not in (0, 1):
                continue
            dx, dy = rotvec(p['x'], -p['y'], rot)
            pt = (round(x + dx, 3), round(y + dy, 3))
            groups.setdefault(pt, []).append(p)
            pinmap[p['num']] = (round(pt[0] - self.ox, 3), round(pt[1] - self.oy, 3))
        used = set()
        for (px, py), plist in groups.items():
            net = 'UNSET'
            for p in plist:
                if p['num'] in nets:
                    net = nets[p['num']]
                    used.add(p['num'])
            if net == 'UNSET':
                if all(p['etype'] == 'no_connect' for p in plist):
                    continue
                raise KeyError('%s %s: pin %s (%s) not assigned' % (ref, lib_id, plist[0]['num'], plist[0]['name']))
            a = math.radians(plist[0]['ang'])
            bx, by = rotvec(math.cos(a), -math.sin(a), rot)
            out = (int(round(-bx)), int(round(-by)))
            if net == '~':
                self.pin_pts.add((px, py))
                continue
            if net is None:
                self._noconn(px, py)
            elif net in POWER:
                self._power(net, px, py, out=out, owner=owner)
            else:
                self._label(net, px, py, dir_to_rot(*out), shapes.get(net, 'bidirectional'), owner)
        unknown = set(nets) - used
        if unknown:
            raise KeyError('%s: unknown pins %s' % (ref, unknown))
        # body bbox: graphics extents, else pin extents shrunk by pin length
        bx0, by0, bx1, by1 = self._body(sdef, pins, x, y, rot)
        self._bb('body', 'body:' + ref, bx0, by0, bx1, by1, owner)
        # fields
        rw, vw = tw(ref), tw(value)
        mode = fields or 'above'
        if isinstance(mode, tuple):
            rp = (x + mode[0], y + mode[1])
            vp = (rp[0], rp[1] + 2.54)
        elif mode == 'hz':
            rp, vp = (x, by0 - 1.905), (x, by1 + 1.905)
        elif mode == 'above':
            rp, vp = (bx0, by0 - 3.81), (bx0, by0 - 1.27)
        elif mode == 'below':
            rp, vp = (bx0, by1 + 2.54), (bx0, by1 + 5.08)
        elif mode == 'right':
            cy = (by0 + by1) / 2
            rp, vp = (bx1 + 1.27, cy - 1.27), (bx1 + 1.27, cy + 1.27)
        elif mode == 'left':
            cy = (by0 + by1) / 2
            rp, vp = (bx0 - 1.27 - max(rw, vw), cy - 1.27), (bx0 - 1.27 - max(rw, vw), cy + 1.27)
        rp = (snap(rp[0], 0.635), snap(rp[1], 0.635))
        vp = (snap(vp[0], 0.635), snap(vp[1], 0.635))
        frot = rot % 180
        # KiCad shows field text at (symbol rot + field rot); with frot = rot%180 the text reads horizontally.
        # A 180-degree display flips 'left' justification, so pick the justification that renders left-aligned.
        j = 'left' if (rot + frot) % 360 == 0 else 'right'
        if mode == 'hz':
            j = None
        node = [S('symbol'), [S('lib_id'), lib_id], [S('at'), x, y, rot], [S('unit'), 1], [S('body_style'), 1],
                [S('exclude_from_sim'), S('no')], [S('in_bom'), S('yes')], [S('on_board'), S('yes')],
                [S('in_pos_files'), S('yes')], [S('dnp'), S('yes' if dnp else 'no')], [S('uuid'), uid()],
                prop('Reference', ref, rp[0], rp[1], rot=frot, justify=j),
                prop('Value', value, vp[0], vp[1], rot=frot, justify=j),
                prop('Footprint', footprint, x, y, hide=True),
                prop('Datasheet', datasheet, x, y, hide=True)]
        for k, v in (extra or {}).items():
            node.append(prop(k, v, x, y, hide=True))
        self._txt_bb(ref, rp[0], rp[1], 0, j or 'center', owner, 'ref:' + ref, kind='field')
        self._txt_bb(value, vp[0], vp[1], 0, j and 'left' or 'center', owner, 'val:' + ref, kind='field')
        seen = set()
        for p in pins:
            if p['num'] not in seen:
                seen.add(p['num'])
                node.append([S('pin'), p['num'], [S('uuid'), uid()]])
        self.items.append(node)
        self.syms.append((node, ref))
        return pinmap

    @staticmethod
    def _body(sdef, pins, x, y, rot):
        xs, ys = [], []
        for sub in find(sdef, 'symbol'):
            for g in sub[2:]:
                if not isinstance(g, list):
                    continue
                if g[0] == 'rectangle':
                    for k in ('start', 'end'):
                        pt = find1(g, k)
                        xs.append(float(pt[1])); ys.append(float(pt[2]))
                elif g[0] in ('polyline', 'arc', 'circle', 'bezier'):
                    for pts in find(g, 'pts'):
                        for xy in find(pts, 'xy'):
                            xs.append(float(xy[1])); ys.append(float(xy[2]))
                    c = find1(g, 'center')
                    if c:
                        r = float(find1(g, 'radius')[1])
                        xs += [float(c[1]) - r, float(c[1]) + r]; ys += [float(c[2]) - r, float(c[2]) + r]
        if not xs:
            xs = [p['x'] for p in pins] or [0]
            ys = [p['y'] for p in pins] or [0]
        pts = [rotvec(a, -b, rot) for a in (min(xs), max(xs)) for b in (min(ys), max(ys))]
        X = [x + p[0] for p in pts]; Y = [y + p[1] for p in pts]
        return min(X), min(Y), max(X), max(Y)

    # ---------------------------------------------------------------- wiring (region-relative)
    def wire(self, *pts):
        P = [(round(self.ox + a, 3), round(self.oy + b, 3)) for a, b in pts]
        for p, q in zip(P, P[1:]):
            if p != q:
                assert p[0] == q[0] or p[1] == q[1], ('non-orthogonal wire', p, q)
                self.segs.append((p, q))
                self._bb('wire', 'wire', p[0] - 0.25, p[1] - 0.25, q[0] + 0.25, q[1] + 0.25, None)

    def glabel(self, net, x, y, rot, shape='bidirectional'):
        X, Y = round(self.ox + x, 3), round(self.oy + y, 3)
        self._own += 1
        self.anchor_pts.add((X, Y))
        self._label(net, X, Y, rot, shape, ('gl', self._own))

    def pwr(self, net, x, y, out=None):
        X, Y = round(self.ox + x, 3), round(self.oy + y, 3)
        self._own += 1
        self.anchor_pts.add((X, Y))
        if out is None:
            out = (0, 1) if net == 'GND' else (0, -1)
        self._power(net, X, Y, out=out, owner=('pw', self._own))

    def _finish_wires(self):
        pts = set(self.pin_pts) | set(self.anchor_pts)
        for p, q in self.segs:
            pts.add(p); pts.add(q)
        out = []
        for p, q in self.segs:
            inner = []
            for r in pts:
                if r in (p, q):
                    continue
                if p[0] == q[0] == r[0] and min(p[1], q[1]) < r[1] < max(p[1], q[1]):
                    inner.append(r)
                elif p[1] == q[1] == r[1] and min(p[0], q[0]) < r[0] < max(p[0], q[0]):
                    inner.append(r)
            chain = sorted([p, q] + inner, key=lambda t: (t[0], t[1]))
            out += list(zip(chain, chain[1:]))
        deg = {}
        for p, q in out:
            deg[p] = deg.get(p, 0) + 1
            deg[q] = deg.get(q, 0) + 1
        for p in list(deg):
            if p in self.pin_pts or p in self.anchor_pts:
                deg[p] += 1
        for p, q in out:
            self.items.append([S('wire'), [S('pts'), [S('xy'), p[0], p[1]], [S('xy'), q[0], q[1]]],
                               [S('stroke'), [S('width'), 0], [S('type'), S('default')]], [S('uuid'), uid()]])
        for p, d in deg.items():
            if d >= 3:
                self.items.append([S('junction'), [S('at'), p[0], p[1]], [S('diameter'), 0],
                                   [S('color'), 0, 0, 0, 0], [S('uuid'), uid()]])
        self.segs = []

    # ---------------------------------------------------------------- layout helpers
    def two_pin(self, lib_id, ref, value, x, y, n1, n2, horizontal=True, footprint='', dnp=False, src=None):
        """horizontal: pin1 left, pin2 right; ref above, value below."""
        if horizontal:
            return self.place(lib_id, ref, value, x, y, {'1': n1, '2': n2}, rot=90, footprint=footprint,
                              dnp=dnp, src=src, fields='hz')
        return self.place(lib_id, ref, value, x, y, {'1': n1, '2': n2}, rot=0, footprint=footprint,
                          dnp=dnp, src=src, fields='right')

    def decap(self, ref, value, x, y, rail, footprint, gnd='GND'):
        return self.place('Device:C_Small', ref, value, x, y, {'1': rail, '2': gnd},
                          footprint=footprint, src=symlib.from_can('Device:C_Small'), fields='right')

    # ---------------------------------------------------------------- checks / output
    def overlaps(self):
        res = []
        B = self.boxes
        for i in range(len(B)):
            ki, di, a0, b0, a1, b1, oi = B[i]
            for j in range(i + 1, len(B)):
                kj, dj, c0, d0, c1, d1, oj = B[j]
                if ki == 'wire' and kj == 'wire':
                    continue
                if 'wire' in (ki, kj) and 'power' in (ki, kj):
                    continue
                if ki == 'body' and kj == 'body':
                    pass
                elif oi is not None and oi == oj and 'field' not in (ki, kj):
                    continue          # a symbol's own labels/power symbols touch its pins
                elif oi is not None and oi == oj and {ki, kj} == {'field'}:
                    pass
                ox = min(a1, c1) - max(a0, c0)
                oy = min(b1, d1) - max(b0, d0)
                if ox > 0.15 and oy > 0.15:
                    res.append((di, dj, round(a0, 1), round(b0, 1)))
        return res

    def outside(self):
        res = []
        for b, bx in zip(self.boxes, self.box_of):
            if bx is None or b[1].startswith('note:' + bx[0][:20]):
                continue
            t, X0, Y0, X1, Y1 = bx
            if b[2] < X0 + 0.5 or b[3] < Y0 + 3.5 or b[4] > X1 - 0.5 or b[5] > Y1 - 0.5:
                res.append((t[:25], b[1], round(b[2], 1), round(b[3], 1), round(b[4], 1), round(b[5], 1)))
        return res

    def tree(self, root_title, comment):
        self._finish_wires()
        lib_symbols = [S('lib_symbols')] + [self.libs[k] for k in sorted(self.libs)]
        for node, ref in self.syms:
            r = ref if ref else find1(node, 'property')[2]
            node.append([S('instances'), [S('project'), PROJECT,
                                          [S('path'), '/' + self.uuid, [S('reference'), r], [S('unit'), 1]]]])
        tb = [S('title_block'), [S('title'), root_title], [S('date'), '2026-11-15'], [S('rev'), 'A'],
              [S('company'), 'Mobot'], [S('comment'), 1, comment]]
        return [S('kicad_sch'), [S('version'), S(VERSION)], [S('generator'), 'eeschema'],
                [S('generator_version'), '10.0'], [S('uuid'), self.uuid],
                [S('paper'), 'User', self.paper[0], self.paper[1]], tb, lib_symbols] + self.items + [
            [S('sheet_instances'), [S('path'), '/', [S('page'), '1']]], [S('embedded_fonts'), S('no')]]


def number_power(sheet):
    n = {'#PWR': 0, '#FLG': 0}
    for node, ref in sheet.syms:
        if ref is None:
            p = find1(node, 'property')
            base = p[2].rstrip('?')
            n[base] += 1
            p[2] = '%s%03d' % (base, n[base])


def write(path, tree):
    with open(path, 'w', encoding='utf-8') as f:
        f.write(dump(tree) + '\n')
