"""Mobot main board schematic (single large sheet). Run: python3 -I design.py <out_dir>"""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import engine, symlib, customsyms
from engine import Sheet

OUT = sys.argv[1]
CUSTOM = os.path.join(OUT, 'mobot.kicad_sym')
customsyms.write(CUSTOM)
CAN = symlib.from_can


def cust(name):
    return symlib.from_file(CUSTOM, name, 'mobot:' + name)


R0402 = 'Resistor_SMD:R_0402_1005Metric'
R0603 = 'Resistor_SMD:R_0603_1608Metric'
R1206 = 'Resistor_SMD:R_1206_3216Metric'
R2010 = 'Resistor_SMD:R_2010_5025Metric'
R2512 = 'Resistor_SMD:R_2512_6332Metric'
C0402 = 'Capacitor_SMD:C_0402_1005Metric'
C0603 = 'Capacitor_SMD:C_0603_1608Metric'
R0603_HS = 'Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder'
C0603_HS = 'Capacitor_SMD:C_0603_1608Metric_Pad1.08x0.95mm_HandSolder'
C0805 = 'Capacitor_SMD:C_0805_2012Metric'
C1206 = 'Capacitor_SMD:C_1206_3216Metric'
SMA = 'Diode_SMD:D_SMA'
TPFP = 'TestPoint:TestPoint_Pad_D1.0mm'
SOT23 = 'Package_TO_SOT_SMD:SOT-23'
SOT235 = 'Package_TO_SOT_SMD:SOT-23-5'
LED0603 = 'LED_SMD:LED_0603_1608Metric'
BTN = 'mobot:SW_GT-TC048D-H025-L1'
WR = '~'      # pin is connected by drawn wires

W = Sheet(paper=(1220, 850))


# ---------------------------------------------------------------- part helpers (region-relative)
def R(ref, val, x, y, n1, n2, fp=R0402, dnp=False):
    """horizontal resistor centred at (x, y), both ends labelled."""
    return W.two_pin('Device:R_US', ref, val, x, y, n1, n2, footprint=fp, dnp=dnp, src=CAN('Device:R_US'))


def C(ref, val, x, y, n1, n2, fp=C0402, dnp=False):
    return W.two_pin('Device:C_Small', ref, val, x, y, n1, n2, footprint=fp, dnp=dnp, src=CAN('Device:C_Small'))


def vC(ref, val, x, ytop, fp=C0402, top=WR, bot='GND', fields='right'):
    """vertical cap whose top pin sits at (x, ytop)."""
    return W.place('Device:C_Small', ref, val, x, ytop + 2.54, {'1': top, '2': bot}, footprint=fp,
                   src=CAN('Device:C_Small'), fields=fields)


def vR(ref, val, x, ytop, top=WR, bot=WR, fp=R0402, fields='right', extra=None):
    return W.place('Device:R_US', ref, val, x, ytop + 3.81, {'1': top, '2': bot}, footprint=fp,
                   src=CAN('Device:R_US'), fields=fields, extra=extra)


def hR(ref, val, xl, y, left=WR, right=WR, fp=R0402):
    return W.place('Device:R_US', ref, val, xl + 3.81, y, {'1': left, '2': right}, rot=90, footprint=fp,
                   src=CAN('Device:R_US'), fields='hz')


def hC(ref, val, xl, y, left=WR, right=WR, fp=C0402):
    return W.place('Device:C_Small', ref, val, xl + 2.54, y, {'1': left, '2': right}, rot=90, footprint=fp,
                   src=CAN('Device:C_Small'), fields='hz')


def hDa(ref, val, xa, y, a=WR, k=WR, fp=SMA, lib='Device:D_Schottky'):
    """horizontal diode, anode pin at (xa, y), cathode to the right."""
    src = CAN(lib) if lib in ('Device:D_Schottky', 'Device:LED') else None
    return W.place(lib, ref, val, xa + 3.81, y, {'1': k, '2': a}, rot=180, footprint=fp, src=src, fields='hz')


def hDk(ref, val, xk, y, k=WR, a=WR, fp=SMA, lib='Device:D_Schottky'):
    """horizontal diode, cathode pin at (xk, y), anode to the right."""
    src = CAN(lib) if lib in ('Device:D_Schottky', 'Device:LED') else None
    return W.place(lib, ref, val, xk + 3.81, y, {'1': k, '2': a}, rot=0, footprint=fp, src=src, fields='hz')


def vDk(ref, val, x, ytop, k=WR, a=WR, fp=SMA, lib='Device:D_Schottky'):
    """vertical diode, cathode on top at (x, ytop)."""
    src = CAN(lib) if lib in ('Device:D_Schottky', 'Device:LED') else None
    return W.place(lib, ref, val, x, ytop + 3.81, {'1': k, '2': a}, rot=270, footprint=fp, src=src, fields='right')


def vDa(ref, val, x, ytop, a=WR, k=WR, fp=SMA, lib='Device:D_Schottky'):
    """vertical diode, anode on top at (x, ytop)."""
    src = CAN(lib) if lib in ('Device:D_Schottky', 'Device:LED') else None
    return W.place(lib, ref, val, x, ytop + 3.81, {'1': k, '2': a}, rot=90, footprint=fp, src=src, fields='right')


def D(ref, val, x, y, anode, cathode, fp=SMA, lib='Device:D_Schottky'):
    return hDa(ref, val, x - 3.81, y, anode, cathode, fp, lib)


def TP(ref, net, x, y):
    return W.place('Connector:TestPoint', ref, net, x, y, {'1': net}, footprint=TPFP, fields='right')


def pflag(x, y):
    """PWR_FLAG on a wired node (no net name)."""
    X, Y = round(W.ox + x, 3), round(W.oy + y, 3)
    W._own += 1
    W.anchor_pts.add((X, Y))
    W._power('', X, Y, flag=True, owner=('pf', W._own))


# =====================================================================================================
# ROW 1 - POWER (wired, CAN-board style; only block inputs/outputs are global labels)
# =====================================================================================================
W.region(20, 20)
W.box(0, 0, 190, 115, 'Battery In / ESC Out')
CAN2 = CAN('Connector_Generic:Conn_01x02')
J = W.place('Connector_Generic:Conn_01x02', 'J1', 'BATT IN', 20.32, 30.48, {'1': 'GND', '2': WR}, rot=180,
            footprint='mobot:AMASS_XT30UPB-F_1x02_P5.0mm_Vertical', src=CAN2, fields=(-2.54, -8.89))
J2 = W.place('Connector_Generic:Conn_01x02', 'J2', 'ESC OUT', 20.32, 55.88, {'1': 'GND', '2': WR}, rot=180,
             footprint='mobot:AMASS_XT30UPB-M_1x02_P5.0mm_Vertical', src=CAN2, fields=(-2.54, -8.89))
yb = J['2'][1]                                    # battery rail
W.wire(J['2'], (116.84, yb))
W.wire((35.56, yb), (35.56, J2['2'][1]), J2['2'])
W.place('Device:D_TVS', 'D1', 'SMBJ12CA', 55.88, yb + 3.81, {'2': WR, '1': 'GND'}, rot=90,
        footprint='Diode_SMD:D_SMB', fields=(5.08, -1.27))
vC('C1', '2.2uF 100V', 71.12, yb, C1206)
vC('C2', '2.2uF 100V', 86.36, yb, C1206)
W.glabel('VBAT', 116.84, yb, 0, 'output')
W.wire((101.6, yb), (101.6, 66.04))
d = hDa('D2', 'SS54', 101.6, 66.04)
W.wire(d['1'], (124.46, 66.04))
W.glabel('VLOGIC', 124.46, 66.04, 0, 'output')
W.glabel('USB_VBUS', 93.98, 83.82, 180, 'input')
d = hDa('D3', 'SS54', 101.6, 83.82)
W.wire((93.98, 83.82), d['2'])
W.wire(d['1'], (116.84, 83.82), (116.84, 66.04))
W.note(5.08, 99.06, ['ESC current: J1 -> J2 only (short wide pours on L1+L4). No fuse on this board.',
                     'VLOGIC = diode-OR of VBAT and USB_VBUS -> 3.3 V regulator (S3 + P4 run from USB alone).',
                     'XT30: 15 A cont / 30 A for 1 min. Confirm launch current (PLAN D8).'])

W.region(220, 20)
W.box(0, 0, 330, 145, 'Voltage Regulation 5V (TPS54360B, CAN-board design)')
U = W.place('Power_Management:TPS54360BQDDARQ1', 'U1', 'TPS54360BQDDARQ1', 101.6, 50.8,
            {'1': WR, '2': WR, '3': WR, '4': WR, '5': WR, '6': WR, '7': 'GND', '8': WR, 'EPAD': 'GND'},
            footprint='mobot:DDA8_3P1X2P4_TEX-L', src=CAN('Power_Management:TPS54360BQDDARQ1'), fields='above')
yv = U['2'][1]
W.glabel('VBAT', 12.7, yv, 180, 'input')
W.wire((12.7, yv), U['2'])
vC('C6', '2.2uF 100V', 25.4, yv, C1206)
r = vR('R1', '174k', 50.8, yv, fp=R0603)
en = r['2']
vR('R2', '43.2k', 50.8, en[1], bot='GND')
W.wire(en, (91.44, en[1]), (91.44, U['3'][1]), U['3'])
r = vR('R3', '162k', 96.52, U['4'][1], bot='GND')
W.wire(U['4'], r['1'])
W.wire(U['1'], (93.98, U['1'][1]), (93.98, 33.02), (119.38, 33.02))
c = hC('C3', '100nF', 119.38, 33.02)
xsw = 177.8
W.wire(c['2'], (xsw, 33.02), (xsw, U['8'][1]))
W.wire(U['8'], (xsw, U['8'][1]))
vDk('D4', 'SS56', xsw, U['8'][1], a='GND')
L = W.place('Device:L', 'L1', '8.2uH', 190.5, U['8'][1], {'1': WR, '2': WR}, rot=90,
            footprint='Inductor_SMD:L_Changjiang_FNR4010S', src=CAN('Device:L'), fields='hz')
W.wire((xsw, U['8'][1]), L['1'])
ypre = U['8'][1]
W.wire(L['2'], (241.3, ypre))
vC('C7', '47uF 10V', 205.74, ypre, C1206)
vC('C8', '47uF 10V', 220.98, ypre, C1206)
d = hDa('D5', 'SS54', 241.3, ypre)
W.wire(d['1'], (274.32, ypre))
W.pwr('+5V', 259.08, ypre)
vC('C9', '4.7uF X7R', 274.32, ypre, C0805)
d = hDk('D6', 'SS54', 259.08, 71.12)
W.wire(d['1'], (259.08, ypre))
W.glabel('USB_VBUS', d['2'][0], 71.12, 0, 'input')
# compensation
W.wire(U['6'], (157.48, U['6'][1]), (157.48, 66.04))
vR('R4', '13.0k', 157.48, 66.04)
vC('C4', '6.8nF', 157.48, 73.66)
W.wire((157.48, 62.23), (167.64, 62.23))
vC('C5', '39pF', 167.64, 62.23)
# feedback divider
yfb = 96.52
W.wire(U['5'], (152.4, U['5'][1]), (152.4, yfb))
vR('R6', '10.2k', 152.4, yfb, bot='GND', fields='left')
r = hR('R5', '59k', 152.4, yfb)
W.wire(r['2'], (233.68, yfb), (233.68, ypre))
W.note(5.08, 127.0, ['UVLO R1/R2: start ~5.8 V, stop ~5.2 V (CAN board R2 = 35.3k gave 6.9 / 6.3 V: too high for 2S).',
                     'FB R5/R6 -> 5.43 V before D5; D5 drops it to ~5.0 V on +5V.',
                     '+5V loads: heater (~0.5 A) + IMU LDO (~45 mA). Firmware: heater off when running from USB only.'])

W.region(560, 20)
W.box(0, 0, 200, 115, 'Voltage Regulation 3.3V (TPSM33620S3)')
U = W.place('Regulator_Linear:TPSM33620S3QRDNRQ1', 'U2', 'TPSM33620S3QRDNRQ1', 76.2, 30.48,
            {'1': None, '2': WR, '3': WR, '4': WR, '8': WR, '9': WR, '10': 'GND', '11': 'GND'},
            footprint='mobot:TPSM33620_RDN0011B', src=CAN('Regulator_Linear:TPSM33620S3QRDNRQ1'), fields='above')
yi = U['3'][1]
W.glabel('VLOGIC', 12.7, yi, 180, 'input')
W.wire((12.7, yi), U['3'])
vC('C11', '4.7uF 25V', 20.32, yi, C0805)
vC('C12', '100nF', 33.02, yi)
r = vR('R7', '100k', 53.34, yi, fields='left')
W.wire(r['2'], (53.34, U['2'][1]), U['2'])
c = hC('C10', '1uF', 68.58, U['8'][1], left='GND')
W.wire(c['2'], U['8'])
pflag(74.93, U['8'][1])
yo = U['4'][1]
W.wire(U['4'], (180.34, yo))
W.wire(U['9'], (129.54, U['9'][1]), (129.54, yo))
vC('C13', '22uF', 142.24, yo, C0603)
vC('C14', '22uF', 154.94, yo, C0805)
vC('C15', '22uF', 167.64, yo, C0805)
W.pwr('+3.3V', 180.34, yo)
W.note(5.08, 106.68, ['VLOGIC 3-36 V. 3.3 V loads: P4 module (P4 + PSRAM + C6 Wi-Fi), S3, camera, encoder, LEDs.'])

W.region(770, 20)
W.box(0, 0, 170, 115, 'Battery Voltage Sense / Power LED / Flags')
W.glabel('VBAT', 12.7, 20.32, 180, 'input')
W.wire((12.7, 20.32), (25.4, 20.32))
r = vR('R8', '39k', 25.4, 20.32)
vR('R9', '22k', 25.4, r['2'][1], bot='GND')
yn = r['2'][1]
W.wire(r['2'], (50.8, yn))
vC('C16', '100nF', 38.1, yn)
W.glabel('VBAT_SENSE', 50.8, yn, 0, 'output')
W.pwr('+3.3V', 25.4, 55.88)
r = vR('R10', '1k', 25.4, 55.88)
W.place('Device:LED', 'D7', 'PWR', 25.4, r['2'][1] + 3.81, {'1': 'GND', '2': WR}, rot=90, footprint=LED0603,
        src=CAN('Device:LED'), fields=(5.08, -1.27))
for i, net in enumerate(['VBAT', 'VLOGIC']):
    W.flag(net, 101.6 + i * 22.86, 27.94)
for i, net in enumerate(['GND', '+5V', '+3.3V']):
    W.flag(net, 93.98 + i * 22.86, 66.04)
W.note(5.08, 106.68, ['VBAT_SENSE = VBAT x 0.361 (8.4 V -> 3.03 V), read directly from the battery input -> S3 IO4.'])

W.region(950, 20)
W.box(0, 0, 150, 115, 'Mounting Holes (GND, double as GND test points) / Rail Test Pads')
for i in range(4):
    W.place('Mechanical:MountingHole_Pad', 'H%d' % (i + 1), 'GND', 12.7 + (i % 2) * 68.58, 25.4 + (i // 2) * 25.4,
            {'1': 'GND'}, footprint='MountingHole:MountingHole_3.2mm_M3_Pad_Via',
            src=CAN('Mechanical:MountingHole_Pad'), fields='right')
for i, net in enumerate(['VBAT', 'VLOGIC', '+5V', '+3.3V']):
    if net != 'VBAT':   # VBAT is probed at the XT30 (its TP was dropped)
        TP('TP%d' % i, net, 12.7 + i * 35.56, 86.36)

# =====================================================================================================
# ROW 2 - S3, USB, LEDs, IMU island
# =====================================================================================================
W.region(20, 180)
W.box(0, 0, 330, 230, 'ESP32')
U = W.place('RF_Module:ESP32-S3-WROOM-1', 'U3', 'ESP32-S3-WROOM-1-N8', 149.86, 99.06, {
    '1': 'GND', '2': WR, '3': 'S3_EN', '4': 'VBAT_SENSE', '5': 'S3_SCH_SCK', '6': 'S3_SCH_MOSI',
    '7': 'SCH_MISO', '8': 'RGB_DIN', '9': 'CAN_D', '10': 'START_LED', '11': 'P4_BOOT', '12': 'ENC_SDA',
    '13': 'S3_USB_DN', '14': 'S3_USB_DP', '15': 'P4_EN', '16': 'START_BTN', '17': 'ENC_SCL', '18': 'SCH_CS',
    '19': 'PWM_ESC', '20': 'PWM_STEER_F', '21': 'PWM_STEER_R', '22': 'PWM_SPARE', '23': 'ISM_CS',
    '24': 'SCH_EXTRESN', '25': 'SYNC', '26': None, '27': 'S3_BOOT', '28': 'S3_ISM_MOSI', '29': 'S3_ISM_SCK',
    '30': 'ISM_MISO', '31': 'ISM_INT1', '32': 'ISM_INT2', '33': 'HEATER_PWM', '34': 'P4_TO_S3',
    '35': 'S3_TO_P4', '36': 'S3_U0RXD', '37': 'S3_U0TXD', '38': 'STATUS_LED', '39': 'CAN_R'},
    footprint='RF_Module:ESP32-S3-WROOM-1', src=CAN('RF_Module:ESP32-S3-WROOM-1'), fields=(-25.4, -36.83),
    shapes={'SYNC': 'output', 'PWM_ESC': 'output', 'PWM_STEER_F': 'output', 'PWM_STEER_R': 'output',
            'PWM_SPARE': 'output', 'HEATER_PWM': 'output', 'P4_EN': 'output', 'P4_BOOT': 'output',
            'S3_TO_P4': 'output', 'P4_TO_S3': 'input', 'VBAT_SENSE': 'input', 'SCH_MISO': 'input',
            'ISM_MISO': 'input', 'ISM_INT1': 'input', 'ISM_INT2': 'input', 'SCH_CS': 'output',
            'ISM_CS': 'output', 'SCH_EXTRESN': 'output', 'START_BTN': 'input', 'START_LED': 'output',
            'CAN_D': 'output', 'CAN_R': 'input'})
p = U['2']
W.wire(p, (p[0], 63.5), (p[0] + 35.56, 63.5))
W.pwr('+3.3V', p[0], 63.5)
vC('C17', '22uF', p[0] + 22.86, 63.5, C0603)
vC('C18', '100nF', p[0] + 35.56, 63.5)
W.place('Device:R_Pack04', 'RN1', '22', 266.7, 73.66, {
    '1': 'S3_SCH_SCK', '8': 'SCH_SCK', '2': 'S3_SCH_MOSI', '7': 'SCH_MOSI', '3': 'S3_ISM_SCK', '6': 'ISM_SCK',
    '4': 'S3_ISM_MOSI', '5': 'ISM_MOSI'}, rot=270,
    footprint='Resistor_SMD:R_Array_Concave_4x0402', src=CAN('Device:R_Pack04'), fields='above')
W.note(215.9, 116.84, ['RN1: 22R source-series on SPI clock/data (place at U3).',
                       'IO45 unused (strapping). IO46 = START, IO17 = START LED, IO0 = BOOT (internal pull-ups).',
                       'S3 Wi-Fi = the only Wi-Fi on the board (P4 C6 unused); off during runs.',
                       'Keep the WROOM-1 antenna at a board edge with no copper under it.'])
R('R11', '10k', 40.64, 160.02, '+3.3V', 'S3_EN')
C('C19', '1uF', 40.64, 177.8, 'S3_EN', 'GND')
W.place('Switch:SW_Push', 'SW1', 'RESET', 40.64, 200.66, {'1': 'S3_EN', '2': 'GND'}, footprint=BTN, fields='hz')
W.place('Switch:SW_Push', 'SW2', 'BOOT', 127.0, 200.66, {'1': 'S3_BOOT', '2': 'GND'}, footprint=BTN, fields='hz')
# START: PC-case style front-panel header (switch on 1-2, LED on 3-4), LED driven straight from IO17
W.place('Connector_Generic:Conn_01x04', 'J4', 'START', 226.06, 195.58,
        {'1': 'START_BTN', '2': 'GND', '3': 'START_LED', '4': 'GND'},
        footprint='Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical', fields='right')
W.note(241.3, 160.02, ['J4 = PC-case style front panel: pins 1-2 START switch, 3-4 LED+ / LED-.',
                       'LED+ driven straight from IO17 (START_LED, 3.3 V); LED- to GND.',
                       'The LED or its harness needs its own series resistor (about 100-150R for a bare LED).'])

W.region(360, 180)
W.box(0, 0, 230, 120, 'USB (S3 native USB; also bridges P4 UART0 for flashing)')
W.place('Connector:USB_C_Receptacle_USB2.0_16P', 'J3', 'USB-C', 30.48, 50.8, {
    'A1': 'GND', 'A4': 'USB_VBUS', 'A5': 'USB_CC1', 'A6': 'S3_USB_DP', 'A7': 'S3_USB_DN', 'A8': None,
    'B5': 'USB_CC2', 'B6': 'S3_USB_DP', 'B7': 'S3_USB_DN', 'B8': None, 'SH': 'GND'},
    footprint='Connector_USB:USB_C_Receptacle_JAE_DX07S016JA1R1500',
    src=CAN('Connector:USB_C_Receptacle_USB2.0_16P'), fields='above')
R('R12', '5.1k', 116.84, 20.32, 'USB_CC1', 'GND')
R('R13', '5.1k', 116.84, 35.56, 'USB_CC2', 'GND')
W.place('Power_Protection:USBLC6-2SC6', 'U4', 'USBLC6-2SC6 DNP', 132.08, 76.2,
        {'1': 'S3_USB_DN', '6': 'S3_USB_DN', '3': 'S3_USB_DP', '4': 'S3_USB_DP', '2': 'GND', '5': 'USB_VBUS'},
        footprint='Package_TO_SOT_SMD:SOT-23-6', src=CAN('Power_Protection:USBLC6-2SC6'), dnp=True,
        fields=(-22.86, -12.7))
W.flag('USB_VBUS', 193.04, 30.48)
W.note(5.08, 111.76, ['U4: footprint only, DNP by default (PLAN D13).'])

W.region(360, 310)
W.box(0, 0, 230, 100, 'Status LEDs (LIFE heartbeat + NeoPixel for status/faults)')
for (ref_r, ref_d, net, name, y) in [('R14', 'D8', 'STATUS_LED', 'LIFE', 38.1)]:
    r = hR(ref_r, '100', 40.64, y, left=net)
    dd = hDa(ref_d, name, 66.04, y, k='GND', fp=LED0603, lib='Device:LED')
    W.wire(r['2'], dd['2'])
U = W.place('LED:WS2812B-2020', 'D9', 'WS2812B-2020', 160.02, 66.04,
            {'1': None, '2': 'GND', '3': 'RGB_DIN', '4': WR},
            footprint='LED_SMD:LED_WS2812B-2020_PLCC4_2.0x2.0mm', src=CAN('LED:WS2812B-2020'), fields=(10.16, 1.27))
p = U['4']
W.wire(p, (p[0], 50.8), (p[0] + 15.24, 50.8))
W.pwr('+3.3V', p[0], 50.8)
vC('C20', '100nF', p[0] + 15.24, 50.8)

W.region(20, 420)
W.box(0, 0, 330, 50, 'Test Pads (bare pads, no pogo)')
for i, net in enumerate(['S3_U0TXD', 'S3_U0RXD', 'SYNC', 'SCH_SCK', 'SCH_MISO', 'ISM_SCK', 'ISM_MISO',
                         'S3_TO_P4', 'P4_TO_S3']):
    if 'U0' not in net and 'P4' not in net:   # UARTs are probed at J5/J7 (their TPs were dropped)
        TP('TP%d' % (i + 2), net, 12.7 + i * 35.56, 17.78)

# ---- IMU island
W.region(600, 180)
W.box(0, 0, 220, 95, 'Island Supply (+5V -> ferrite -> AP2112K-3.3)')
W.pwr('+5V', 12.7, 30.48)
fb = W.place('Device:FerriteBead_Small', 'FB1', '600R@100MHz', 22.86, 30.48, {'1': WR, '2': WR}, rot=90,
             footprint='Inductor_SMD:L_0603_1608Metric', fields='hz')
W.wire((12.7, 30.48), fb['1'])
U = W.place('Regulator_Linear:AP2112K-3.3', 'U5', 'AP2112K-3.3', 109.22, 33.02,
            {'1': WR, '3': WR, '2': 'GND', '5': WR}, footprint=SOT235, fields='above')
W.wire(fb['2'], U['1'])
W.wire((96.52, U['1'][1]), (96.52, U['3'][1]), U['3'])
vC('C21', '10uF', 50.8, 30.48, C0805)
pflag(38.1, 30.48)
yo = U['5'][1]
W.wire(U['5'], (190.5, yo))
vC('C22', '10uF', 137.16, yo, C0805)
vC('C23', '100nF', 152.4, yo)
W.place('Connector:TestPoint', 'TP9', '+3V3_IMU', 167.64, yo, {'1': WR}, footprint=TPFP, fields='right')
W.glabel('+3V3_IMU', 190.5, yo, 0, 'output')
W.note(5.08, 88.9, ['AP2112K-3.3 (on hand): Vin <= 6 V, so it is fed from +5V, never VBAT.'])

W.region(600, 285)
W.box(0, 0, 220, 135, 'ISM6HG256X (backup gyro, SPI2) - CAN-board circuit')
U = W.place('Sensor_Motion:ISM6HG256XTR', 'U7', 'ISM6HG256XTR', 60.96, 25.4, {
    '1': 'ISM_MISO', '2': 'GND', '3': 'GND', '4': 'ISM_INT1', '5': WR, '6': 'GND', '7': 'GND',
    '8': WR, '9': 'ISM_INT2', '10': None, '11': None, '12': 'ISM_CS', '13': 'ISM_SCK', '14': 'ISM_MOSI'},
    footprint='mobot:LGA14L_2P5X3_STM', src=CAN('Sensor_Motion:ISM6HG256XTR'), fields='above')
vdd, vio = U['8'], U['5']
W.wire(vio, (vio[0] - 5.08, vio[1]), (vio[0] - 5.08, vdd[1]))
W.wire(vdd, (17.78, vdd[1]))
vC('C29', '100nF', 45.72, vdd[1])
vC('C30', '100nF', 33.02, vdd[1])
W.glabel('+3V3_IMU', 17.78, vdd[1], 180, 'input')

W.region(830, 180)
W.box(0, 0, 360, 145, 'SCH16T-K01 (primary gyro, SafeSPI on SPI3)')
U = W.place('mobot:SCH16T-K01', 'U6', 'SCH16T-K01-004', 119.38, 63.5, {
    '1': 'GND', '3': 'GND', '4': 'GND', '5': 'GND', '6': 'SCH_EXTRESN', '7': 'GND', '8': WR, '9': 'GND',
    '10': WR, '11': WR, '12': 'GND', '13': 'GND', '14': WR, '15': WR,
    '16': 'GND', '17': WR, '18': WR, '19': 'SCH_MISO', '20': WR, '21': 'SCH_SCK',
    '22': 'SCH_CS', '23': 'SCH_MOSI', '24': 'GND'},
    footprint='mobot:SCH16T_SOIC-24_Murata', src=cust('SCH16T-K01'), fields='above',
    datasheet='mainboard/datasheets/02_imu/Datasheet-SCH16T-K01_JAN2024.pdf')
# left: V3P3 pin 8 -> C27 (C4), VREGA pins 10/11 -> C25 (C2)
p8 = U['8']
W.wire(p8, (60.96, p8[1]))
vC('C27', '100nF', 73.66, p8[1], fields='left')
W.glabel('+3V3_IMU', 60.96, p8[1], 180, 'input')
W.wire(U['10'], (96.52, U['10'][1]), (96.52, U['11'][1]))
W.wire(U['11'], (91.44, U['11'][1]), (91.44, 83.82))
vC('C25', '10uF', 91.44, 83.82, C0805, fields='left')
# right: VDDIO 18 + V3P3 17 -> C28 (C5) + C26 (C3); VREGD 14/15 -> C24 (C1)
p18, p17 = U['18'], U['17']
W.wire(p17, (p17[0] + 2.54, p17[1]), (p18[0] + 2.54, p18[1]))
W.wire(p18, (195.58, p18[1]))
vC('C26', '1uF', 162.56, p18[1])
vC('C28', '100nF', 177.8, p18[1])
W.glabel('+3V3_IMU', 195.58, p18[1], 0, 'input')
W.wire(U['15'], (147.32, U['15'][1]))
W.wire(U['14'], (147.32, U['14'][1]))
W.wire((147.32, U['15'][1]), (147.32, 83.82))
vC('C24', '1uF', 147.32, 83.82)
# DRY_SYNC <- SYNC through R15
p20 = U['20']
W.wire(p20, (152.4, p20[1]))
r = hR('R15', '0R', 152.4, p20[1])
W.glabel('SYNC', r['2'][0], p20[1], 0, 'input')
W.note(5.08, 116.84, ['Caps per datasheet Table 80: C24 = C1 (VREGD), C25 = C2 (VREGA), C26 = C3 (V3P3 pin 17), C27 = C4 (V3P3 pin 8),',
                      'C28 = C5 (VDDIO). Place each at its pin on the top layer, ESR < 100 mOhm. SPI mode 0, <= 10.5 MHz.',
                      'DRY_SYNC used as SYNC input (S3 = time master). TA8/TA9 = GND, pins 5/7 GND, pin 2 open, no center pad.'])

W.region(830, 335)
W.box(0, 0, 360, 155, 'Heater (4 x 39R, one on each side of SCH16T)')
ytop, ybot = 20.32, 27.94
W.pwr('+5V', 17.78, ytop)
W.wire((17.78, ytop), (25.4 + 3 * 30.48, ytop))
for i in range(4):
    vR('R%d' % (16 + i), '39R 1W', 25.4 + i * 30.48, ytop, fp=R2512,
       extra={'Manufacturer': 'Yageo', 'MPN': 'RC2512FK-0739RL'})
Q = W.place('Transistor_FET:AO3400A', 'Q1', 'AO3400A', 144.78, 45.72, {'1': WR, '2': 'GND', '3': WR},
            footprint=SOT23, fields='right')
W.wire((25.4, ybot), (Q['3'][0], ybot), Q['3'])
gate = (127.0, Q['1'][1])
W.wire(Q['1'], gate)
vR('R21', '100k', gate[0], gate[1], bot='GND')
r = hR('R20', '1k', gate[0] - 7.62, gate[1])
W.glabel('HEATER_PWM', r['1'][0], gate[1], 180, 'input')
W.note(5.08, 137.16, ['4 x 39R in parallel = 9.75R -> 2.6 W at 5 V (0.64 W each, 2512 = 1 W). One per side of U6, bottom side.',
                      'No hardware cutoff: R21 holds the heater off whenever the S3 is in reset or not driving HEATER_PWM.',
                      'Firmware: watchdog on, duty cap, SCH16T vs ISM6HG256X temperature cross-check. HEATER_PWM >= 20 kHz or a few Hz.'])

# =====================================================================================================
# ROW 3 - P4, microSD, antenna, camera, IR, actuation
# =====================================================================================================
W.region(20, 500)
W.box(0, 0, 360, 215, 'ESP32-P4 Module (Waveshare: P4NRW32; on-module C6 unused)')
p4nets = {'1': 'GND', '2': None, '4': None, '5': None, '9': None, '10': None,
          '84': WR, '85': WR, '86': WR, '87': 'P4_EN', '17': 'SYNC', '18': None,
          '22': 'CAM_SCL', '23': 'CAM_SDA', '24': 'CAM_PWR_EN',
          '41': 'CSI_D0_N', '42': 'CSI_D0_P', '43': 'CSI_CLK_P', '44': 'CSI_CLK_N', '45': 'CSI_D1_N',
          '46': 'CSI_D1_P', '50': 'P4_USBJ_DN', '51': 'P4_USBJ_DP', '62': 'P4_BOOT', '63': 'P4_BOOT_EN',
          '64': 'P4_TO_S3', '65': 'S3_TO_P4', '66': WR, '67': 'SD_D0', '68': 'SD_D1', '69': 'SD_D2',
          '70': 'SD_D3', '71': 'SD_CLK', '72': 'SD_CMD', '73': 'SD_DET'}
for n in [6, 7, 8, 11, 12, 13, 14, 16, 19, 20, 21, 25, 26, 27, 28, 29, 30, 31, 32, 34, 35, 36, 37, 38, 39, 48, 49,
          53, 54, 55, 56, 57, 58, 59, 60, 61, 74, 75, 76, 77, 78, 79, 80, 81, 82, 88]:
    p4nets[str(n)] = None
U = W.place('mobot:Waveshare_ESP32-P4-Module', 'U8', 'ESP32-P4-Module', 124.46, 93.98, p4nets,
            footprint='mobot:Waveshare_ESP32-P4-Module', src=cust('Waveshare_ESP32-P4-Module'), fields='above',
            datasheet='https://www.waveshare.com/wiki/ESP32-P4-Module',
            shapes={'P4_TO_S3': 'output', 'S3_TO_P4': 'input', 'P4_EN': 'input', 'P4_BOOT': 'input',
                    'SYNC': 'input', 'CAM_PWR_EN': 'output'})
p85, p84 = U['85'], U['84']
xs = p85[0] - 2.54
W.wire(p85, (xs, p85[1]), (xs, p84[1]), p84)
W.wire((xs, p85[1]), (45.72, p85[1]))
vC('C31', '10uF', 76.2, p85[1], C0805)
vC('C32', '100nF', 63.5, p85[1])
W.pwr('+3.3V', 45.72, p85[1])
p66 = U['66']
W.wire(p66, (185.42, p66[1]))
vC('C34', '1uF', 172.72, p66[1])
W.glabel('P4_VO4', 185.42, p66[1], 0, 'output')
R('R22', '10k', 238.76, 22.86, '+3.3V', 'P4_EN')
C('C33', '1uF', 238.76, 38.1, 'P4_EN', 'GND')
R('R23', '10k', 238.76, 68.58, '+3.3V', 'P4_BOOT_EN')
for i, net in enumerate(['P4_USBJ_DN', 'P4_USBJ_DP']):
    TP('TP%d' % (10 + i), net, 228.6 + (i % 3) * 38.1, 127.0 + (i // 3) * 30.48)
W.note(5.08, 185.42, ['C6 co-processor unused (Wi-Fi comes from the S3): LNA_OUT open; firmware holds C6 in reset via GPIO54.',
                     'Do NOT use: GPIO6 (pad 21) + GPIO54 (pad 82) = C6 link; GPIO0/1 (pads 88/16) = 32 kHz xtal on module.',
                     'Boot: GPIO35 low at EN rise = download (GPIO36 pulled up). UART0 = GPIO37 TX / GPIO38 RX.',
                     'GPIO39-48 (*) run at ESP_LDO_VO4: set VO4 = 3.3 V (default-speed SD) or 1.8 V (UHS-I).',
                     'SD slot 0 IOMUX: CLK 43, CMD 44, D0-D3 39-42 (verify vs P4 TRM before layout).'])

W.region(390, 500)
W.box(0, 0, 220, 215, 'microSD (P4 SDMMC slot 0, 4-bit) - pull-ups to P4_VO4 at the socket')
J = W.place('Connector:Micro_SD_Card_Det1', 'J6', 'microSD', 172.72, 55.88, {
    '1': WR, '2': WR, '3': WR, '4': WR, '5': 'SD_CLK', '6': 'GND', '7': WR, '8': WR, '9': WR, 'SH': 'GND'},
    footprint='Connector_Card:microSD_HC_Hirose_DM3AT-SF-PEJM5', fields='above')
xr_up, xr_dn = 30.48, 83.82
# upper group: DAT2, DAT3, CMD - resistors hang from the rail above
for (pin, net, ref, col) in [('1', 'SD_D2', 'R27', 139.7), ('2', 'SD_D3', 'R28', 119.38), ('3', 'SD_CMD', 'R24', 99.06)]:
    pp = J[pin]
    W.wire(pp, (col - 5.08, pp[1]))
    W.glabel(net, col - 5.08, pp[1], 180)
    r = vR(ref, '10k', col, xr_up, fields='right')
    W.wire(r['2'], (col, pp[1]))
W.wire((88.9, xr_up), (139.7, xr_up))
W.glabel('P4_VO4', 88.9, xr_up, 180, 'input')
# lower group: DAT0, DAT1, DET - resistors stand on the rail below
for (pin, net, ref, col) in [('9', 'SD_DET', 'R29', 144.78), ('8', 'SD_D1', 'R26', 124.46), ('7', 'SD_D0', 'R25', 104.14)]:
    pp = J[pin]
    W.wire(pp, (col - 5.08, pp[1]))
    W.glabel(net, col - 5.08, pp[1], 180)
    r = vR(ref, '10k', col, xr_dn - 7.62, fields='right')
    W.wire(r['1'], (col, pp[1]))
W.wire((93.98, xr_dn), (144.78, xr_dn))
W.glabel('P4_VO4', 93.98, xr_dn, 180, 'input')
# VDD with its decoupling
pv = J['4']
W.wire(pv, (60.96, pv[1]))
vC('C35', '10uF', 78.74, pv[1], C0805)
vC('C36', '100nF', 68.58, pv[1], C0603_HS)
W.pwr('+3.3V', 60.96, pv[1])
W.note(5.08, 124.46, ['Pull-ups go to P4_VO4 (the SD I/O domain), not +3.3V.'])

W.region(620, 500)
W.box(0, 0, 280, 150, 'Camera: Arducam OV9281 NoIR M12 (Pi 15-pin 1.0 mm FPC, 2-lane)')
J = W.place('Connector_Generic:Conn_01x15', 'J8', 'CAMERA', 33.02, 63.5, {
    '1': 'GND', '2': 'CSI_D0_N', '3': 'CSI_D0_P', '4': 'GND', '5': 'CSI_D1_N', '6': 'CSI_D1_P', '7': 'GND',
    '8': 'CSI_CLK_N', '9': 'CSI_CLK_P', '10': 'GND', '11': 'CAM_PWR_EN', '12': None, '13': WR,
    '14': WR, '15': WR}, rot=180,
    footprint='mobot:Molex_200528-0150_1x15-1MP_P1.00mm_Horizontal', fields='above')
p = J['15']
y3 = 33.02
W.wire(p, (p[0] + 5.08, p[1]), (p[0] + 5.08, y3), (116.84, y3))
W.pwr('+3.3V', p[0] + 5.08, y3)
r = vR('R31', '2.2k', 55.88, y3)
W.wire(r['2'], (55.88, J['14'][1]))
W.wire(J['14'], (60.96, J['14'][1]))
W.glabel('CAM_SDA', 60.96, J['14'][1], 0)
r = vR('R30', '2.2k', 81.28, y3)
W.wire(r['2'], (81.28, J['13'][1]))
W.wire(J['13'], (86.36, J['13'][1]))
W.glabel('CAM_SCL', 86.36, J['13'][1], 0)
vC('C37', '10uF', 104.14, y3, C0805)
vC('C38', '100nF', 116.84, y3)
R('R32', '1k', 152.4, 93.98, 'SYNC', 'CAM_TRIG')
R('R33', '1.2k DNP', 152.4, 109.22, 'CAM_TRIG', 'GND', dnp=True)
W.place('Connector_Generic:Conn_01x02', 'J9', 'CAM TRIG', 236.22, 93.98, {'1': 'CAM_TRIG', '2': 'GND'},
        footprint='Connector_JST:JST_SH_BM02B-SRSS-TB_1x02-1MP_P1.00mm_Vertical',
        src=CAN('Connector_Generic:Conn_01x02'), fields='right')
W.note(5.08, 137.16, ['Pi FPC: pin 11 = camera power enable, 12 unused. Trigger wire to the camera board XVS/FSIN pad.',
                      'R33 fitted -> 1.8 V trigger level (if the Arducam pad is 1.8 V).'])

W.region(620, 660)
W.box(0, 0, 280, 130, 'PWM_Connectors (3x4 servo header; servos on VBAT, ESC signal only)')
# RN2 pairs run 4-5 (ESC) .. 1-8 (SPARE) so the S3 pins (IO11-14 top to bottom), the array and the
# header SIG column (ESC at the top) route as one uncrossed bus.
W.place('Device:R_Pack04', 'RN2', '33', 35.56, 43.18, {
    '4': 'PWM_ESC', '3': 'PWM_STEER_F', '2': 'PWM_STEER_R', '1': 'PWM_SPARE',
    '5': 'SIG_ESC', '6': 'SIG_STEER_F', '7': 'SIG_STEER_R', '8': 'SIG_SPARE'}, rot=270,
    footprint='Resistor_SMD:R_Array_Concave_4x0402', src=CAN('Device:R_Pack04'), fields='above')
H = W.place('mobot:Servo_Header_3x04', 'J10', 'SERVO', 91.44, 41.91, {
    '1': 'SIG_ESC', '4': 'SIG_STEER_F', '7': 'SIG_STEER_R', '10': 'SIG_SPARE', '2': None, '5': WR, '8': WR,
    '11': WR, '3': WR, '6': WR, '9': WR, '12': WR},
    footprint='mobot:PinHeader_3x04_P2.54mm_Servo', src=cust('Servo_Header_3x04'), fields='above')
xb = 119.38
for p in ('5', '8', '11'):
    W.wire(H[p], (xb, H[p][1]))
W.wire((xb, H['11'][1]), (xb, 27.94), (213.36, 27.94))
for i, ref in enumerate(('C39', 'C40', 'C41', 'C42')):
    vC(ref, '22uF 25V X5R', 152.4 + 15.24 * i, 27.94, fp='Capacitor_SMD:C_1210_3225Metric')
W.glabel('VBAT', 213.36, 27.94, 0, 'input')
for p in ('3', '6', '9', '12'):
    W.wire(H[p], (xb, H[p][1]))
W.wire((xb, H['3'][1]), (xb, H['12'][1]))
W.pwr('GND', xb, H['12'][1])
W.note(5.08, 111.76, ['J10: one 3x4 0.1in header (columns ESC / STEER F / STEER R / SPARE, rows SIG / V+ / GND) for a stacked servo pin block.',
                      'ESC V+ (pin 2) intentionally open (no BEC back-feed). Servo VBAT traces >= 1 mm; C39-C42 at the split.',
                      'No board electrolytic: the ESC input cap (on XT30 out) provides the bulk and hot-plug damping. Bench without the ESC: TVS D1 clamps.',
                      'goBILDA 5-turn servo must be reprogrammed before steering use (PLAN D11). SPARE can drive an external IR light.'])

W.region(910, 660)
W.box(0, 0, 280, 130, 'Encoder (Qwiic)')
W.place('Connector_Generic:Conn_01x04', 'J11', 'ENCODER', 50.8, 33.02,
        {'1': 'GND', '2': '+3.3V', '3': 'ENC_SDA', '4': 'ENC_SCL'},
        footprint='Connector_JST:JST_SH_BM04B-SRSS-TB_1x04-1MP_P1.00mm_Vertical', fields='right')
R('R34', '4.7k', 165.1, 27.94, '+3.3V', 'ENC_SDA')
R('R35', '4.7k', 165.1, 43.18, '+3.3V', 'ENC_SCL')
W.note(5.08, 121.92, ['J11 = AS5600 encoder (I2C0, 400 kHz, address 0x36).'])

W.region(920, 510)
W.box(0, 0, 260, 125, 'Debug UART0 (JST-GH, DNP, bottom side)')
GH = 'Connector_JST:JST_GH_SM03B-GHS-TB_1x03-1MP_P1.25mm_Horizontal'
W.place('Connector_Generic:Conn_01x03', 'J5', 'S3 UART', 45.72, 40.64,
        {'1': 'GND', '2': 'S3_U0RXD', '3': 'S3_U0TXD'}, footprint=GH, dnp=True, fields='right')
W.place('Connector_Generic:Conn_01x03', 'J7', 'P4 UART', 170.18, 40.64,
        {'1': 'GND', '2': 'S3_TO_P4', '3': 'P4_TO_S3'}, footprint=GH, dnp=True, fields='right')
W.note(5.08, 88.9, ['JST-GH 1.25 mm, 3 pins on both: 1 GND, 2 RX (into the chip), 3 TX (from the chip).',
                    'Adapter wiring: adapter TX -> pin 2, adapter RX <- pin 3. No power pin: power the board normally.',
                    'J7 sits on the P4<->S3 UART link: hold the S3 in reset (or tri-state IO41/IO42) before driving it.',
                    'Both DNP: footprints only, place on B.Cu.'])

W.region(391, 726)
W.box(0, 0, 220, 110, 'CAN (SN65HVD232, copied from the Rolly CAN+IMU board; all DNP, bottom side)')
W.place('Interface_CAN_LIN:SN65HVD232', 'U9', 'SN65HVD232DR', 45.72, 43.18,
        {'1': 'CAN_D', '2': 'GND', '3': '+3.3V', '4': 'CAN_R', '6': 'CAN-', '7': 'CAN+'},
        footprint='Package_SO:SOIC-8_3.9x4.9mm_P1.27mm', dnp=True,
        shapes={'CAN_D': 'input', 'CAN_R': 'output'}, fields='above')
C('C43', '100nF', 45.72, 78.74, '+3.3V', 'GND', fp=C0603_HS, dnp=True)
GH4 = 'Connector_JST:JST_GH_SM04B-GHS-TB_1x04-1MP_P1.25mm_Horizontal'
W.place('Connector_Generic:Conn_01x04', 'CAN0', 'CAN0', 116.84, 33.02,
        {'1': None, '2': 'CAN+', '3': 'CAN-', '4': 'GND'}, footprint=GH4, dnp=True, fields='right')
W.place('Connector_Generic:Conn_01x04', 'CAN1', 'CAN1', 116.84, 66.04,
        {'1': None, '2': 'CAN+', '3': 'CAN-', '4': 'GND'}, footprint=GH4, dnp=True, fields='right')
R('R36', '120', 182.88, 30.48, 'CAN-', 'CAN_TERM', fp=R0603_HS, dnp=True)
W.two_pin('Jumper:SolderJumper_2_Open', 'CAN_TERM', 'CAN 120R', 182.88, 48.26, 'CAN_TERM', 'CAN+',
          footprint='Jumper:SolderJumper-2_P1.3mm_Open_RoundedPad1.0x1.5mm', dnp=True,
          src=symlib.from_stock('Jumper:SolderJumper_2_Open'))
W.note(5.08, 99.06, ['TWAI: CAN_D = IO16 (TX), CAN_R = IO1 (RX). CAN0/CAN1 in parallel, DroneCAN pinout: 1 +5V (left open), 2 CANH, 3 CANL, 4 GND.',
                     'Bridge CAN_TERM to add the 120R terminator (only at a bus end). Everything here is DNP, on B.Cu.'])

W.region(0, 0)
W.cur_box = None
W.note(20.32, 740.0, ['Mobot main board - single-sheet schematic. Blocks connect through global labels (CAN+IMU board style);',
                      'regulators, heater and decoupling are drawn with wires. Power symbols: +3.3V, +5V, GND.',
                      'Design decisions and open items: mainboard/PLAN.md'])

# =====================================================================================================
engine.number_power(W)
engine.write(os.path.join(OUT, 'mobot_mainboard.kicad_sch'),
             W.tree('Mobot main board', 'ESP32-S3 control + Waveshare ESP32-P4-Module vision; OSH Park 4-layer'))
count = W.nets_used
print('label nets:', len(count), 'single-use:', sorted(k for k, v in count.items() if v < 2))
ov = W.overlaps()
print('overlaps:', len(ov))
for o in ov:
    print('  ', o)
out = W.outside()
print('outside boxes:', len(out))
for o in out:
    print('  ', o)
