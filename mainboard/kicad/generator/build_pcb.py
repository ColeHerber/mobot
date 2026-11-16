"""Sync mobot_mainboard.kicad_pcb to the schematic netlist (in place, preserving the existing hand placement)
and lay out the voltage-regulator clusters per their datasheet layout examples.

  /Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/3.9/bin/python3.9 -I \
      generator/build_pcb.py <netlist.xml> <board.kicad_pcb>

- Footprints removed from the schematic are deleted; footprints whose library footprint changed are swapped at
  the same position/rotation/side; new parts are parked to the right of everything.
- Every pad's net is re-assigned from the netlist (extra pads such as thermal vias inherit the pad they sit in).
- Only the three regulator clusters are moved, each anchored at its IC's current position.  No routing.
"""
import sys, os, xml.etree.ElementTree as ET
import pcbnew

NETLIST, BOARD = sys.argv[1], sys.argv[2]
HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
STOCK = '/Applications/KiCad/KiCad.app/Contents/SharedSupport/footprints'
mm = pcbnew.FromMM

board = pcbnew.LoadBoard(BOARD)
board.SetCopperLayerCount(4)     # OSH Park 4-layer (PLAN D12)

# ---------------------------------------------------------------- netlist
root = ET.parse(NETLIST).getroot()
pin_net = {}
for n in root.find('nets'):
    name = n.get('name')
    ni = board.FindNet(name)
    if ni is None:
        ni = pcbnew.NETINFO_ITEM(board, name)
        board.Add(ni)
    for node in n:
        pin_net[(node.get('ref'), node.get('pin'))] = ni

comps = {}
for c in root.find('components'):
    ts = c.find('tstamps')
    comps[c.get('ref')] = (c.findtext('value'), c.findtext('footprint'), ts.text.strip() if ts is not None and ts.text else '')


def load_fp(fpid):
    lib, name = fpid.split(':', 1)
    path = os.path.join(HERE, 'mobot.pretty') if lib == 'mobot' else os.path.join(STOCK, lib + '.pretty')
    fp = pcbnew.FootprintLoad(path, name)
    if fp is None:
        raise SystemExit('cannot load footprint %s' % fpid)
    fp.SetFPIDAsString(fpid)
    return fp


existing = {f.GetReference(): f for f in board.GetFootprints()}
log = []
for ref, f in list(existing.items()):
    if ref not in comps:
        board.Remove(f)
        del existing[ref]
        log.append('removed ' + ref)

parked = 0
right_edge = max((pcbnew.ToMM(f.GetPosition().x) for f in existing.values()), default=0) + 20
fps = {}
for ref, (val, fpid, uuid) in comps.items():
    old = existing.get(ref)
    fp = None
    if old is not None and old.GetFPIDAsString() == fpid:
        fp = old
    else:
        fp = load_fp(fpid)
        if old is not None:
            fp.SetPosition(old.GetPosition())
            if old.IsFlipped():
                fp.Flip(fp.GetPosition(), pcbnew.FLIP_DIRECTION_LEFT_RIGHT)
            fp.SetOrientation(old.GetOrientation())
            board.Remove(old)
            log.append('swapped %s -> %s' % (ref, fpid))
        else:
            fp.SetPosition(pcbnew.VECTOR2I(mm(right_edge), mm(20 + parked * 10)))
            parked += 1
            log.append('added ' + ref)
        board.Add(fp)
    fp.SetReference(ref)
    fp.SetValue(val)
    if uuid:
        fp.SetPath(pcbnew.KIID_PATH('/' + uuid))
    fp.SetSheetname('/')
    fp.SetSheetfile('mobot_mainboard.kicad_sch')
    numbered = {}
    for p in fp.Pads():
        ni = pin_net.get((ref, p.GetNumber()))
        p.SetNet(ni if ni is not None else board.FindNet(''))
        if ni is not None:
            numbered[p.GetNumber()] = p
    for p in fp.Pads():
        if p.GetNetCode() > 0:
            continue
        for q in numbered.values():
            if q.HitTest(p.GetPosition()):
                p.SetNet(q.GetNet())
                break
    fps[ref] = fp

# ---------------------------------------------------------------- placement helpers
DIRS = {'left': (-1, 0), 'right': (1, 0), 'up': (0, -1), 'down': (0, 1)}


def at(ref, x, y, rot=0, side='F'):
    fp = fps[ref]
    if (side == 'B') != fp.IsFlipped():
        fp.Flip(fp.GetPosition(), pcbnew.FLIP_DIRECTION_LEFT_RIGHT)
    fp.SetOrientationDegrees(rot)
    fp.SetPosition(pcbnew.VECTOR2I(mm(x), mm(y)))


def pad_xy(ref, num):
    for p in fps[ref].Pads():
        if p.GetNumber() == num:
            q = p.GetPosition()
            return pcbnew.ToMM(q.x), pcbnew.ToMM(q.y)
    raise KeyError((ref, num))


def put(ref, pad, x, y, pad_dir, side='F'):
    """Place a 2-terminal part so `pad` lands on (x, y) and points `pad_dir` away from the part centre."""
    want = DIRS[pad_dir]
    for rot in (0, 90, 180, 270):
        at(ref, 0, 0, rot, side)
        px, py = pad_xy(ref, pad)
        cx, cy = pcbnew.ToMM(fps[ref].GetPosition().x), pcbnew.ToMM(fps[ref].GetPosition().y)
        dx, dy = px - cx, py - cy
        s = max(abs(dx), abs(dy), 1e-9)
        if (round(dx / s), round(dy / s)) == want:
            at(ref, x - dx, y - dy, rot, side)
            return
    raise RuntimeError('no orientation for %s pad %s %s' % (ref, pad, pad_dir))


def anchor(ref):
    p = fps[ref].GetPosition()
    return pcbnew.ToMM(p.x), pcbnew.ToMM(p.y)


# ---------------------------------------------------------------- 5 V buck: TPS54360B (datasheet Fig. 53)
# BOOT/VIN/EN/RT left, SW/GND/COMP/FB right (as in the figure); input cap at VIN, catch diode and inductor
# tight on SW, boot cap over the top, RT at its pin, compensation then FB divider to the right away from SW.
X, Y = anchor('U101')
at('U101', X, Y, 0)
vin = pad_xy('U101', '2'); sw = pad_xy('U101', '8'); boot = pad_xy('U101', '1')
en = pad_xy('U101', '3'); rt = pad_xy('U101', '4'); comp = pad_xy('U101', '6'); fb = pad_xy('U101', '5')
put('C106', '1', vin[0] - 2.4, vin[1], 'down')
put('C103', '1', boot[0] + 0.8, boot[1] - 2.6, 'left')
put('D104', '1', sw[0] + 2.2, sw[1] - 4.4, 'right')
put('L101', '1', sw[0] + 3.6, sw[1] - 0.4, 'down')
put('C107', '1', sw[0] + 8.0, sw[1] - 7.2, 'left')
put('C108', '1', sw[0] + 8.0, sw[1] - 10.0, 'left')
put('R101', '1', en[0] - 5.4, en[1] - 3.2, 'up')
put('R102', '1', en[0] - 5.4, en[1] + 1.2, 'up')
put('R103', '1', rt[0] - 0.6, rt[1] + 2.2, 'up')
put('C105', '1', comp[0] + 2.6, comp[1] - 0.2, 'up')
put('R104', '1', comp[0] + 4.0, comp[1] + 0.1, 'left')
put('C104', '1', comp[0] + 6.6, comp[1] + 0.2, 'up')
put('R106', '1', fb[0] + 9.6, fb[1] + 0.6, 'up')
put('R105', '2', fb[0] + 9.6, fb[1] - 0.6, 'down')
put('D105', '2', sw[0] + 12.6, sw[1] - 7.2, 'left')
put('C109', '1', sw[0] + 19.6, sw[1] - 7.2, 'left')
put('D106', '1', sw[0] + 17.2, sw[1] - 11.4, 'right')

# ---------------------------------------------------------------- 3.3 V module: TPSM33620S3 (datasheet Fig. 8-20 / 8-21)
X, Y = anchor('U102')
at('U102', X, Y, 0)
vin = pad_xy('U102', '3'); vcc = pad_xy('U102', '8'); en = pad_xy('U102', '2'); vout = pad_xy('U102', '4')
put('C112', '1', vin[0] - 0.3, vin[1], 'right', side='B')   # 100 nF on the bottom, through the via nearest VIN
put('C111', '1', vin[0] - 1.5, vin[1], 'right')             # 4.7 uF against VIN/GND
put('R107', '2', en[0] - 1.8, en[1] - 1.1, 'right')
put('C110', '2', vcc[0] + 1.2, vcc[1], 'left')              # VCC 1 uF at the pin
put('C113', '1', vout[0] - 0.2, vout[1] + 2.6, 'up')        # output caps straight off VOUT, away from SW
put('C114', '1', vout[0] - 2.8, vout[1] + 2.8, 'up')
put('C115', '1', vout[0] - 5.4, vout[1] + 2.8, 'up')

# ---------------------------------------------------------------- IMU LDO: AP2112K-3.3 (caps at the pins)
X, Y = anchor('U301')
at('U301', X, Y, 0)
vin = pad_xy('U301', '1'); vo = pad_xy('U301', '5')
put('C301', '1', vin[0] - 1.8, vin[1], 'right')
put('FB301', '2', vin[0] - 5.0, vin[1], 'right')
put('C302', '1', vo[0] + 1.8, vo[1], 'left')
put('C303', '1', vo[0] + 1.6, vo[1] + 1.8, 'up')
put('TP301', '1', vo[0] + 5.6, vo[1], 'left') if False else None

pcbnew.SaveBoard(BOARD, board)
print('\n'.join(log))
print('footprints:', len(fps))
