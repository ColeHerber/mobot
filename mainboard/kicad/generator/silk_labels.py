"""Chip silk labels via a 'Label' field (Value keeps the part number for the BOM), and every silk text
   at OSH Park's minimum: 0.8 mm tall, 0.127 mm (5 mil) stroke.
   sch: python3 silk_labels.py sch <in> <out>     pcb: kicad-python silk_labels.py pcb <in> <out>"""
import sys, re
LABEL = {'U1': '5V BUCK', 'U2': '3V3 REG', 'U3': 'ESP32-S3', 'U5': 'IMU LDO', 'U6': 'SCH16T GYRO',
         'U7': 'ISM6 IMU', 'U8': 'ESP32-P4', 'Q1': 'HEATER FET', 'D1': 'TVS', 'D9': 'RGB',
         'D7': 'PWR', 'D8': 'LIFE', 'L1': '5V IND', 'FB1': 'FB', 'RN1': 'SPI 22R', 'RN2': 'PWM 33R'}
H, TH = 0.8, 0.127
mode, src, dst = sys.argv[1:4]
if mode == 'sch':
    sys.path.insert(0, __import__('os').path.dirname(__import__('os').path.abspath(__file__)))
    from schtop import spans, info
    T = open(src).read()
    edits = []
    for s, e, n in spans(T):
        d = info(n)
        r = d.get('Reference') if d['kind'] == 'symbol' else None
        if r not in LABEL: continue
        blk = T[s:e]
        blk = re.sub(r'\n\t\t\(property "Label" .*?\n\t\t\)', '', blk, flags=re.S)
        m = re.search(r'\n\t\t\(property "Datasheet" "[^"]*"\n\t\t\t\(at ([^)]*)\)', blk)
        prop = ('\n\t\t(property "Label" "%s"\n\t\t\t(at %s)\n\t\t\t(hide yes)\n\t\t\t(show_name no)\n\t\t\t(do_not_autoplace no)'
                '\n\t\t\t(effects\n\t\t\t\t(font\n\t\t\t\t\t(size 1.27 1.27)\n\t\t\t\t)\n\t\t\t)\n\t\t)') % (LABEL[r], m.group(1))
        blk = blk[:m.start()] + prop + blk[m.start():]
        edits.append((s, e, blk))
    for s, e, nb in sorted(edits, reverse=True):
        T = T[:s] + nb + T[e:]
    open(dst, 'w').write(T)
    print('sch labels:', len(edits))
else:
    import pcbnew
    b = pcbnew.LoadBoard(src)
    M = pcbnew.FromMM
    n = 0
    for f in b.GetFootprints():
        r = f.GetReference()
        if r in LABEL:
            back = f.GetLayer() == pcbnew.B_Cu
            R = f.Reference()
            fl = f.GetField('Label')
            if fl is None:
                fl = pcbnew.PCB_FIELD(f, pcbnew.FIELD_T_USER, 'Label'); f.Add(fl)
            fl.SetText(LABEL[r]); fl.SetVisible(True)
            fl.SetLayer(pcbnew.B_SilkS if back else pcbnew.F_SilkS)
            fl.SetPosition(R.GetPosition()); fl.SetTextAngle(R.GetTextAngle()); fl.SetMirrored(back); fl.SetKeepUpright(True)
            R.SetLayer(pcbnew.B_Fab if back else pcbnew.F_Fab)
            n += 1
    k = 0
    texts = []
    for f in b.GetFootprints():
        texts += list(f.GetFields()) + [i for i in f.GraphicalItems() if isinstance(i, pcbnew.PCB_TEXT)]
    texts += [d for d in b.Drawings() if isinstance(d, pcbnew.PCB_TEXT)]
    for t in texts:
        if t.GetLayer() in (pcbnew.F_SilkS, pcbnew.B_SilkS):
            t.SetTextSize(pcbnew.VECTOR2I(M(H), M(H))); t.SetTextThickness(M(TH)); k += 1
    b.Save(dst)
    print('pcb labels:', n, 'silk texts resized:', k)
