"""Write a simple AP214 STEP model made of coloured axis-aligned boxes (for parts with no vendor model).

    python3 -I generator/stepbox.py   -> writes 3d/SW_GT-TC048D-H025-L1.step
"""
import os

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class Step:
    def __init__(self):
        self.lines = []
        self.n = 0

    def add(self, s):
        self.n += 1
        self.lines.append('#%d=%s;' % (self.n, s))
        return '#%d' % self.n

    def pt(self, x, y, z):
        return self.add("CARTESIAN_POINT('',(%.4f,%.4f,%.4f))" % (x, y, z))

    def dirn(self, x, y, z):
        return self.add("DIRECTION('',(%.1f,%.1f,%.1f))" % (x, y, z))

    def box(self, x0, y0, z0, x1, y1, z1):
        """closed shell of an axis-aligned box; returns the MANIFOLD_SOLID_BREP id."""
        P = {}
        for i, x in enumerate((x0, x1)):
            for j, y in enumerate((y0, y1)):
                for k, z in enumerate((z0, z1)):
                    P[i, j, k] = self.add('VERTEX_POINT(\'\',%s)' % self.pt(x, y, z))
        coords = {(i, j, k): ((x0, x1)[i], (y0, y1)[j], (z0, z1)[k]) for i in (0, 1) for j in (0, 1) for k in (0, 1)}
        edges = {}

        def edge(a, b):
            key = tuple(sorted((a, b)))
            if key not in edges:
                pa, pb = coords[key[0]], coords[key[1]]
                d = [pb[t] - pa[t] for t in range(3)]
                L = max(abs(v) for v in d)
                vec = self.add("VECTOR('',%s,%.4f)" % (self.dirn(*[v / L for v in d]), L))
                line = self.add("LINE('',%s,%s)" % (self.pt(*pa), vec))
                edges[key] = self.add("EDGE_CURVE('',%s,%s,%s,.T.)" % (P[key[0]], P[key[1]], line))
            return edges[key], key[0] == a

        faces = []
        # each face: axis, side, corner loop ordered counter-clockwise seen from outside
        loops = [
            ((0, 0, 0), (0, 1, 0), (1, 1, 0), (1, 0, 0), (0, 0, -1)),   # bottom
            ((0, 0, 1), (1, 0, 1), (1, 1, 1), (0, 1, 1), (0, 0, 1)),    # top
            ((0, 0, 0), (1, 0, 0), (1, 0, 1), (0, 0, 1), (0, -1, 0)),   # front (y0)
            ((0, 1, 0), (0, 1, 1), (1, 1, 1), (1, 1, 0), (0, 1, 0)),    # back (y1)
            ((0, 0, 0), (0, 0, 1), (0, 1, 1), (0, 1, 0), (-1, 0, 0)),   # left (x0)
            ((1, 0, 0), (1, 1, 0), (1, 1, 1), (1, 0, 1), (1, 0, 0)),    # right (x1)
        ]
        for c0, c1, c2, c3, nrm in loops:
            oes = []
            for a, b in ((c0, c1), (c1, c2), (c2, c3), (c3, c0)):
                e, fwd = edge(a, b)
                oes.append(self.add("ORIENTED_EDGE('',*,*,%s,%s)" % (e, '.T.' if fwd else '.F.')))
            loop = self.add("EDGE_LOOP('',(%s))" % ','.join(oes))
            bound = self.add("FACE_OUTER_BOUND('',%s,.T.)" % loop)
            ref = coords[c0]
            ax = [1, 0, 0] if nrm[0] == 0 else [0, 1, 0]
            axp = self.add("AXIS2_PLACEMENT_3D('',%s,%s,%s)" % (self.pt(*ref), self.dirn(*nrm), self.dirn(*ax)))
            plane = self.add("PLANE('',%s)" % axp)
            faces.append(self.add("ADVANCED_FACE('',(%s),%s,.T.)" % (bound, plane)))
        shell = self.add("CLOSED_SHELL('',(%s))" % ','.join(faces))
        return self.add("MANIFOLD_SOLID_BREP('',%s)" % shell)


def write(path, name, boxes):
    """boxes: [((x0, y0, z0, x1, y1, z1), (r, g, b)), ...] in mm, KiCad footprint axes (y down is fine: STEP y up)."""
    s = Step()
    solids, styles = [], []
    for (b, rgb) in boxes:
        x0, y0, z0, x1, y1, z1 = b
        sol = s.box(x0, -y1, z0, x1, -y0, z1)          # footprint y points down; STEP y points up
        col = s.add("COLOUR_RGB('',%.3f,%.3f,%.3f)" % rgb)
        fac = s.add("FILL_AREA_STYLE_COLOUR('',%s)" % col)
        fsty = s.add("FILL_AREA_STYLE('',(%s))" % fac)
        sfa = s.add("SURFACE_STYLE_FILL_AREA(%s)" % fsty)
        sss = s.add("SURFACE_SIDE_STYLE('',(%s))" % sfa)
        ssa = s.add("SURFACE_STYLE_USAGE(.BOTH.,%s)" % sss)
        psa = s.add("PRESENTATION_STYLE_ASSIGNMENT((%s))" % ssa)
        styles.append(s.add("STYLED_ITEM('',(%s),%s)" % (psa, sol)))
        solids.append(sol)
    unc = s.add("UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-07),#UNIT_MM,'distance_accuracy_value','confusion accuracy')")
    ctx = s.add("(GEOMETRIC_REPRESENTATION_CONTEXT(3) GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((%s)) " % unc +
                "GLOBAL_UNIT_ASSIGNED_CONTEXT((#UNIT_MM,#UNIT_RAD,#UNIT_SR)) REPRESENTATION_CONTEXT('',''))")
    origin = s.add("AXIS2_PLACEMENT_3D('',%s,%s,%s)" % (s.pt(0, 0, 0), s.dirn(0, 0, 1), s.dirn(1, 0, 0)))
    rep = s.add("ADVANCED_BREP_SHAPE_REPRESENTATION('%s',(%s,%s),%s)" % (name, ','.join(solids), origin, ctx))
    s.add("MECHANICAL_DESIGN_GEOMETRIC_PRESENTATION_REPRESENTATION('',(%s),%s)" % (','.join(styles), ctx))
    app = s.add("APPLICATION_CONTEXT('automotive_design')")
    s.add("APPLICATION_PROTOCOL_DEFINITION('international standard','automotive_design',2000,%s)" % app)
    pc = s.add("PRODUCT_CONTEXT('',%s,'mechanical')" % app)
    prod = s.add("PRODUCT('%s','%s','',(%s))" % (name, name, pc))
    pdf = s.add("PRODUCT_DEFINITION_FORMATION('','',%s)" % prod)
    pdc = s.add("PRODUCT_DEFINITION_CONTEXT('part definition',%s,'design')" % app)
    pd = s.add("PRODUCT_DEFINITION('design','',%s,%s)" % (pdf, pdc))
    pds = s.add("PRODUCT_DEFINITION_SHAPE('','',%s)" % pd)
    s.add("SHAPE_DEFINITION_REPRESENTATION(%s,%s)" % (pds, rep))
    body = '\n'.join(s.lines)
    units = ("#UNIT_MM=(LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.));\n"
             "#UNIT_RAD=(NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.));\n"
             "#UNIT_SR=(NAMED_UNIT(*) SI_UNIT($,.STERADIAN.) SOLID_ANGLE_UNIT());")
    # STEP ids must be numeric: give the units the next free numbers
    k = s.n
    for tag in ('#UNIT_MM', '#UNIT_RAD', '#UNIT_SR'):
        k += 1
        body = body.replace(tag, '#%d' % k)
        units = units.replace(tag, '#%d' % k)
    with open(path, 'w') as f:
        f.write("ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION(('%s'),'2;1');\n"
                "FILE_NAME('%s','2026-11-15T00:00:00',(''),(''),'mobot stepbox','','');\n"
                "FILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }'));\nENDSEC;\nDATA;\n%s\n%s\nENDSEC;\n"
                "END-ISO-10303-21;\n" % (name, os.path.basename(path), body, units))


if __name__ == '__main__':
    # G-Switch GT-TC048D-H025-L1: body 4.2 x 3.2 x 1.55, actuator 3.0 x 2.2 to 2.5 mm, 4 leads 0.55 wide to 4.5 mm span
    BODY, CAP, LEAD = (0.80, 0.80, 0.78), (0.93, 0.87, 0.70), (0.85, 0.85, 0.88)
    parts = [((-2.1, -1.6, 0.0, 2.1, 1.6, 1.55), BODY), ((-1.5, -1.1, 1.55, 1.5, 1.1, 2.5), CAP)]
    for sx in (-1, 1):
        for sy in (-1, 1):
            x0, x1 = sorted((sx * 2.1, sx * 2.25))
            parts.append(((x0, sy * 1.075 - 0.275, 0.0, x1, sy * 1.075 + 0.275, 0.5), LEAD))
    write(os.path.join(HERE, '3d', 'SW_GT-TC048D-H025-L1.step'), 'SW_GT-TC048D-H025-L1', parts)
    print('wrote 3d/SW_GT-TC048D-H025-L1.step')
