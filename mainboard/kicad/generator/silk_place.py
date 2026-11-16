"""Auto-place footprint silkscreen fields (reference / value / Label) clear of pads, other silk, part bodies
and the board edge.  Greedy, functional labels first, big parts first.  For each label, at each text size
(largest first) try inside the part's own body; only if no size fits inside, try around it.  LEDs may also
go on the opposite side, under the part.  Labels with no clean spot are left where they were and reported.

    <kicad python> generator/silk_place.py <in.kicad_pcb> <out.kicad_pcb> [keep.txt]

keep.txt: lines "REF FIELD" (e.g. "J1 Value") for hand-placed labels to leave alone; they are avoided.

Run it last, after placement; rerun after moving parts.  Fixed footprint silk (outlines, pin-1 marks,
polarity '+', the servo header letters) is never moved, only avoided.
"""
import sys, math
import pcbnew

SRC, DST = sys.argv[1:3]
KEEP = set(tuple(l.split(None, 1)) for l in open(sys.argv[3]).read().splitlines() if l.strip()) if len(sys.argv) > 3 else set()
M, MM = pcbnew.FromMM, pcbnew.ToMM
PAD_GAP = 0.15      # silk to exposed copper / mask opening
SILK_GAP = 0.15     # silk to other silk
EDGE_GAP = 0.30     # silk to board edge
BODY_GAP = 0.05     # silk to another part's body (its Fab outline; courtyard minus margin if none)
STEP = 0.25         # candidate spacing along a side
SIZES = ((0.8, 0.127), (0.7, 0.12), (0.6, 0.1), (0.5, 0.1))   # (height = width, stroke), tried in order
REACH = (0.1, 0.4, 0.8, 1.2, 2.0, 3.0)   # distance out from the body to try, nearest first
REACH_LABEL = REACH + (4.0, 5.0)       # functional labels may sit a little further out

b = pcbnew.LoadBoard(SRC)
SILK = {pcbnew.F_SilkS: pcbnew.F_Cu, pcbnew.B_SilkS: pcbnew.B_Cu}


class Grid:
    """bbox store with a 2 mm bucket grid; boxes are (x0, y0, x1, y1, owner) in mm."""
    def __init__(self):
        self.cells = {}

    def keys(self, x0, y0, x1, y1):
        for i in range(int(math.floor(x0 / 2)), int(math.floor(x1 / 2)) + 1):
            for j in range(int(math.floor(y0 / 2)), int(math.floor(y1 / 2)) + 1):
                yield i, j

    def add(self, box):
        for k in self.keys(*box[:4]):
            self.cells.setdefault(k, []).append(box)

    def hit(self, x0, y0, x1, y1, gap, skip=None):
        for k in self.keys(x0 - gap, y0 - gap, x1 + gap, y1 + gap):
            for bx in self.cells.get(k, ()):
                if skip is not None and bx[4] is skip:
                    continue
                if x0 - gap < bx[2] and bx[0] < x1 + gap and y0 - gap < bx[3] and bx[1] < y1 + gap:
                    return True
        return False


def mmbox(bb, owner=None):
    return (MM(bb.GetLeft()), MM(bb.GetTop()), MM(bb.GetRight()), MM(bb.GetBottom()), owner)


def silk_boxes(item):
    """thin boxes for a silk graphic, so a part outline doesn't block its whole interior."""
    if isinstance(item, pcbnew.PCB_SHAPE) and item.GetShape() == pcbnew.SHAPE_T_RECT and not item.IsAnyFill():
        x0, y0, x1, y1, _ = mmbox(item.GetBoundingBox())
        w = MM(item.GetWidth()) / 2
        return [(x0, y0, x1, y0 + 2 * w), (x0, y1 - 2 * w, x1, y1), (x0, y0, x0 + 2 * w, y1), (x1 - 2 * w, y0, x1, y1)]
    if isinstance(item, pcbnew.PCB_SHAPE) and item.GetShape() == pcbnew.SHAPE_T_CIRCLE and not item.IsAnyFill():
        c, r = item.GetCenter(), MM(item.GetRadius())
        cx, cy = MM(c.x), MM(c.y)
        return [(cx + r * math.cos(a) - 0.1, cy + r * math.sin(a) - 0.1, cx + r * math.cos(a) + 0.1,
                 cy + r * math.sin(a) + 0.1) for a in [i * math.pi / 8 for i in range(16)]]
    return [mmbox(item.GetBoundingBox())[:4]]


# ---------------------------------------------------------------- obstacles per silk side
pads, silk, bodies = {}, {}, {}
for s in SILK:
    pads[s], silk[s], bodies[s] = Grid(), Grid(), Grid()

movable = []        # (footprint, field)
for f in b.GetFootprints():
    side_cu = f.GetLayer()
    for p in f.Pads():
        for s, cu in SILK.items():
            # only exposed copper matters: tented via pads (thermal vias, hole via rings) can sit under silk
            if p.IsOnLayer(pcbnew.F_Mask if cu == pcbnew.F_Cu else pcbnew.B_Mask):
                pads[s].add(mmbox(p.GetBoundingBox(), f))
    for s, cu in SILK.items():
        fab = pcbnew.F_Fab if cu == pcbnew.F_Cu else pcbnew.B_Fab
        shapes = [g for g in f.GraphicalItems() if g.GetLayer() == fab and isinstance(g, pcbnew.PCB_SHAPE)]
        if shapes:
            bb = shapes[0].GetBoundingBox()
            for g in shapes[1:]:
                bb.Merge(g.GetBoundingBox())
            bodies[s].add(mmbox(bb, f))
            continue
        cy = f.GetCourtyard(pcbnew.F_CrtYd if cu == pcbnew.F_Cu else pcbnew.B_CrtYd)
        if cy.OutlineCount():
            bb = cy.BBox()
            bb.Inflate(-M(0.25))
            bodies[s].add(mmbox(bb, f))
    for g in f.GraphicalItems():
        if g.GetLayer() in SILK and not isinstance(g, pcbnew.PCB_FIELD) and (not isinstance(g, pcbnew.PCB_TEXT) or g.IsVisible()):
            for bx in silk_boxes(g):
                silk[g.GetLayer()].add(bx + (f,))
    for fld in f.GetFields():
        if fld.IsVisible() and fld.GetLayer() in SILK:
            if (f.GetReference(), fld.GetName()) in KEEP:
                silk[fld.GetLayer()].add(mmbox(fld.GetBoundingBox(), None))
            else:
                movable.append((f, fld))
for d in b.Drawings():
    if d.GetLayer() in SILK:
        for bx in silk_boxes(d):
            silk[d.GetLayer()].add(bx + (None,))

edge = pcbnew.SHAPE_POLY_SET()
b.GetBoardPolygonOutlines(edge, False)
inner = pcbnew.SHAPE_POLY_SET(edge)
inner.Deflate(M(EDGE_GAP), pcbnew.CORNER_STRATEGY_ROUND_ALL_CORNERS, M(0.05))


def inside_board(x0, y0, x1, y1):
    return all(inner.Contains(pcbnew.VECTOR2I(M(x), M(y))) for x, y in ((x0, y0), (x1, y0), (x0, y1), (x1, y1)))


def own_box(f, s=None):
    """the part's body (Fab outline): a courtyard can be far bigger, e.g. the WROOM antenna keep-out."""
    cu = f.GetLayer()
    fab = pcbnew.F_Fab if cu == pcbnew.F_Cu else pcbnew.B_Fab
    shapes = [g for g in f.GraphicalItems() if g.GetLayer() == fab and isinstance(g, pcbnew.PCB_SHAPE)]
    if shapes:
        bb = shapes[0].GetBoundingBox()
        for g in shapes[1:]:
            bb.Merge(g.GetBoundingBox())
        return mmbox(bb)[:4]
    cy = f.GetCourtyard(pcbnew.F_CrtYd if cu == pcbnew.F_Cu else pcbnew.B_CrtYd)
    bb = cy.BBox() if cy.OutlineCount() else f.GetBoundingBox(False, False)
    return mmbox(bb)[:4]


def text_size(fld, ang):
    fld.SetTextAngleDegrees(ang)
    fld.SetPosition(pcbnew.VECTOR2I(0, 0))
    x0, y0, x1, y1, _ = mmbox(fld.GetBoundingBox())
    return x1 - x0, y1 - y0, (x0 + x1) / 2, (y0 + y1) / 2   # w, h, and bbox-centre offset from the anchor


def candidates(f, fld, s):
    """(score, angle, cx, cy, w, h): inside the body first, then around it, nearest first."""
    X0, Y0, X1, Y1 = own_box(f)
    fx, fy = (X0 + X1) / 2, (Y0 + Y1) / 2
    out = []
    long_x = (X1 - X0) >= (Y1 - Y0)
    for ang in (0, 90):
        w, h, _, _ = text_size(fld, ang)
        # reading along the part's long side is preferred
        pref = 0 if (ang == 0) == long_x else 0.4
        # inside the body
        nx, ny = int(max(0, X1 - X0 - w) / 2 / 0.2), int(max(0, Y1 - Y0 - h) / 2 / 0.2)
        if w <= X1 - X0 and h <= Y1 - Y0:
            for i in range(-nx, nx + 1):
                for j in range(-ny, ny + 1):
                    cx, cy = fx + i * 0.2, fy + j * 0.2
                    out.append((-10 + math.hypot(cx - fx, cy - fy) * 0.15 + pref, ang, cx, cy, w, h))
        for ri, r in enumerate(REACH if fld.GetName() == 'Reference' else REACH_LABEL):
            spots = []
            # above / below: slide along x
            n = int(max(0, (X1 - X0 + w) / 2) / STEP)
            for k in range(-n, n + 1):
                cx = fx + k * STEP
                spots += [(cx, Y0 - r - h / 2), (cx, Y1 + r + h / 2)]
            # left / right: slide along y
            n = int(max(0, (Y1 - Y0 + h) / 2) / STEP)
            for k in range(-n, n + 1):
                cy = fy + k * STEP
                spots += [(X0 - r - w / 2, cy), (X1 + r + w / 2, cy)]
            for cx, cy in spots:
                d = math.hypot(cx - fx, cy - fy)
                out.append((ri * 2 + d * 0.15 + pref, ang, cx, cy, w, h))
    out.sort(key=lambda c: c[0])
    return out


def clean(s, f, x0, y0, x1, y1):
    return (inside_board(x0, y0, x1, y1)
            and not pads[s].hit(x0, y0, x1, y1, PAD_GAP)
            and not silk[s].hit(x0, y0, x1, y1, SILK_GAP)
            and not bodies[s].hit(x0, y0, x1, y1, BODY_GAP, skip=f))


def area(f):
    X0, Y0, X1, Y1 = own_box(f)
    return (X1 - X0) * (Y1 - Y0)


# Functional labels (Label field, or a Value shown on silk) claim space before plain refdes; within a tier,
# big parts first (they have the fewest legal spots near them).
order = sorted(movable, key=lambda m: (m[1].GetName() == 'Reference', -area(m[0])))
def is_led(f):
    n = f.GetFPIDAsString()
    return 'LED' in n or 'WS2812' in n


def try_side(f, fld, s, inside):
    """inside=True: only spots within the part's body (score < 0); False: only around it."""
    back = s == pcbnew.B_SilkS
    fld.SetLayer(s)
    fld.SetMirrored(back)
    for hgt, th in SIZES:
        fld.SetTextSize(pcbnew.VECTOR2I(M(hgt), M(hgt)))
        fld.SetTextThickness(M(th))
        for sc, ang, cx, cy, w, h in candidates(f, fld, s):
            if (sc < 0) == inside and clean(s, f, cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2):
                return ang, cx, cy, w, h
    return None


placed, hidden, flipped = 0, [], []
for f, fld in order:
    s0 = fld.GetLayer()
    other = pcbnew.B_SilkS if s0 == pcbnew.F_SilkS else pcbnew.F_SilkS
    fld.SetKeepUpright(True)
    saved = (fld.GetPosition(), fld.GetTextAngleDegrees(), fld.GetTextSize(), fld.GetTextThickness(), fld.IsMirrored())
    # inside the part at any size beats outside it; LEDs may then use the other side, inside first again
    tries = [(s0, True), (s0, False)] + ([(other, True), (other, False)] if is_led(f) else [])
    best = None
    for s, inside in tries:
        best = try_side(f, fld, s, inside)
        if best:
            if s != s0:
                flipped.append(f.GetReference())
            break
    if best is None:            # no clean spot: leave it as it was, to fix by hand
        fld.SetLayer(s0)
        fld.SetPosition(saved[0]); fld.SetTextAngleDegrees(saved[1]); fld.SetTextSize(saved[2])
        fld.SetTextThickness(saved[3]); fld.SetMirrored(saved[4])
        hidden.append('%s %s' % (f.GetReference(), fld.GetText()))
        silk[s0].add(mmbox(fld.GetBoundingBox(), None))
        continue
    ang, cx, cy, w, h = best
    _, _, ox, oy = text_size(fld, ang)
    fld.SetPosition(pcbnew.VECTOR2I(M(cx - ox), M(cy - oy)))
    silk[s].add((cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2, None))
    placed += 1

b.Save(DST)
sizes = {}
for f, fld in order:
    sizes[round(MM(fld.GetTextHeight()), 2)] = sizes.get(round(MM(fld.GetTextHeight()), 2), 0) + 1
print('placed %d, left in place %d, sizes %s, LEDs moved to the other side: %s' % (placed, len(hidden), sizes, flipped))
for h in hidden:
    print('  unmoved:', h)
