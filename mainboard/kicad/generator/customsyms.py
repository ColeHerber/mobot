"""Project symbols: SCH16T-K01 and Waveshare ESP32-P4-Module."""
from sexp import Sym as S, dump

LIB = 'mobot'


def _font():
    return [S('effects'), [S('font'), [S('size'), 1.27, 1.27]]]


def _prop(name, val, x, y, hide=False, justify=None):
    p = [S('property'), name, val, [S('at'), x, y, 0]]
    if hide:
        p.append([S('hide'), S('yes')])
    e = _font()
    if justify:
        e.append([S('justify'), S(justify)])
    p += [[S('show_name'), S('no')], [S('do_not_autoplace'), S('no')], e]
    return p


def _pin(etype, x, y, ang, name, num, hide=False):
    p = [S('pin'), S(etype), S('line'), [S('at'), x, y, ang], [S('length'), 2.54]]
    if hide:
        p.append([S('hide'), S('yes')])
    p += [[S('name'), name, _font()], [S('number'), num, _font()]]
    return p


def _symbol(name, ref, value, fp, ds, desc, x0, y0, x1, y1, pins):
    return [S('symbol'), name,
            [S('pin_names'), [S('offset'), 1.016]],
            [S('exclude_from_sim'), S('no')], [S('in_bom'), S('yes')], [S('on_board'), S('yes')],
            [S('in_pos_files'), S('yes')], [S('duplicate_pin_numbers_are_jumpers'), S('no')],
            _prop('Reference', ref, x0, y0 + 1.27, justify='left'),
            _prop('Value', value, x0, y1 - 1.27, justify='left'),
            _prop('Footprint', fp, 0, 0, hide=True),
            _prop('Datasheet', ds, 0, 0, hide=True),
            _prop('Description', desc, 0, 0, hide=True),
            [S('symbol'), name + '_0_1',
             [S('rectangle'), [S('start'), x0, y0], [S('end'), x1, y1],
              [S('stroke'), [S('width'), 0.254], [S('type'), S('default')]],
              [S('fill'), [S('type'), S('background')]]]],
            [S('symbol'), name + '_1_1'] + pins,
            [S('embedded_fonts'), S('no')]]


def sch16t():
    L = []
    left = [('1', 'HEATSINK', 'passive'), ('2', 'N.C.', 'no_connect'), ('3', 'TA9', 'input'),
            ('4', 'TA8', 'input'), ('5', 'RESERVED', 'passive'), ('6', 'EXTRESN', 'input'),
            ('7', 'RESERVED', 'passive'), ('8', 'V3P3', 'power_in'), ('9', 'GND', 'power_in'),
            ('10', 'VREGA2', 'passive'), ('11', 'VREGA', 'passive'), ('12', 'GND', 'power_in')]
    right = [('24', 'HEATSINK', 'passive'), ('23', 'MOSI', 'input'), ('22', 'CS', 'input'),
             ('21', 'SCK', 'input'), ('20', 'DRY_SYNC', 'bidirectional'), ('19', 'MISO', 'tri_state'),
             ('18', 'VDDIO', 'power_in'), ('17', 'V3P3', 'power_in'), ('16', 'GND', 'power_in'),
             ('15', 'VREGD', 'passive'), ('14', 'VREGD2', 'passive'), ('13', 'GND', 'power_in')]
    for i, (n, nm, t) in enumerate(left):
        L.append(_pin(t, -17.78, 13.97 - i * 2.54, 0, nm, n))
    for i, (n, nm, t) in enumerate(right):
        L.append(_pin(t, 17.78, 13.97 - i * 2.54, 180, nm, n))
    return _symbol('SCH16T-K01', 'U', 'SCH16T-K01', 'mobot:SCH16T_SOIC-24_Murata',
                   'mainboard/datasheets/02_imu/Datasheet-SCH16T-K01_JAN2024.pdf',
                   'Murata 6-DoF gyro/accel, SafeSPI v2.0, SOIC-24 (no center pad)',
                   -15.24, 16.51, 15.24, -16.51, L)


P4_NAMES = {1: 'GND', 2: 'LNA_OUT', 3: 'GND', 4: 'C6_U0RXD', 5: 'C6_U0TXD', 6: 'C6_IO15', 7: 'C6_IO14',
            8: 'C6_IO13', 9: 'C6_IO9/BOOT', 10: 'C6_IO8/BOOT_EN', 11: 'C6_IO7', 12: 'C6_IO6', 13: 'C6_IO5',
            14: 'C6_IO4', 15: 'GND', 16: 'GPIO1/XTAL32K_P', 33: 'GND',
            34: 'DSI_DATAP1', 35: 'DSI_DATAN1', 36: 'DSI_CLKN', 37: 'DSI_CLKP', 38: 'DSI_DATAP0',
            39: 'DSI_DATAN0', 40: 'GND', 41: 'CSI_DATAN0', 42: 'CSI_DATAP0', 43: 'CSI_CLKP', 44: 'CSI_CLKN',
            45: 'CSI_DATAN1', 46: 'CSI_DATAP1', 47: 'GND', 48: 'USB_DM', 49: 'USB_DP',
            50: 'GPIO24/USB_FS_DN', 51: 'GPIO25/USB_FS_DP', 52: 'GND', 66: 'ESP_LDO_VO4', 83: 'GND',
            84: 'VBAT', 85: 'ESP_3V3', 86: 'ESP_3V3', 87: 'ESP_EN', 88: 'GPIO0/XTAL32K_N'}
for _i, _g in enumerate(range(2, 14)):
    P4_NAMES[17 + _i] = 'GPIO%d' % _g
for _i, _g in enumerate(range(20, 24)):
    P4_NAMES[29 + _i] = 'GPIO%d' % _g
for _i, _g in enumerate(range(26, 39)):
    P4_NAMES[53 + _i] = 'GPIO%d' % _g
for _i, _g in enumerate(range(39, 55)):
    P4_NAMES[67 + _i] = 'GPIO%d' % _g
P4_NAMES[62] = 'GPIO35/BOOT'
P4_NAMES[63] = 'GPIO36/BOOT_EN'
P4_NAMES[64] = 'GPIO37/U0TXD'
P4_NAMES[65] = 'GPIO38/U0RXD'
P4_NAMES[21] = 'GPIO6/C6_WAKE'
P4_NAMES[82] = 'GPIO54/C6_EN'
P4_NAMES[73] = 'GPIO45'
for _p in range(67, 77):
    P4_NAMES[_p] = P4_NAMES[_p] + '*'   # VDD_IO_5 domain (VO4)


def p4_module():
    assert len(P4_NAMES) == 88
    L = []

    def t(n):
        nm = P4_NAMES[n]
        if nm == 'GND' or nm in ('ESP_3V3', 'VBAT'):
            return 'power_in'
        if nm == 'ESP_LDO_VO4':
            return 'power_out'
        if n == 87:
            return 'input'
        if n in (2, 48, 49) or nm.startswith(('DSI', 'CSI')):
            return 'passive'
        return 'bidirectional'

    # left column (top -> bottom), None = gap
    left = [85, 86, None, 84, None, 87, None, 88, 16] + list(range(17, 33)) + [None, 50, 51, None] + list(range(53, 66))
    right = [2, None] + list(range(4, 15)) + [None] + list(range(34, 40)) + [None] + list(range(41, 47)) + \
            [None, 48, 49, None, 66, None] + list(range(67, 83))
    top = 2.54 * (max(len(left), len(right)) // 2)
    y = top
    for n in left:
        if n is not None:
            stacked = n == 86
            L.append(_pin('passive' if stacked else t(n), -25.4, y if not stacked else y + 2.54, 0, P4_NAMES[n], str(n), hide=stacked))
            if stacked:
                continue
        y -= 2.54
    y = top
    for n in right:
        if n is not None:
            L.append(_pin(t(n), 25.4, y, 180, P4_NAMES[n], str(n)))
        y -= 2.54
    gnd = [1, 3, 15, 33, 40, 47, 52, 83]
    bot = y - 2.54
    for i, n in enumerate(gnd):
        L.append(_pin('passive' if i > 0 else 'power_in', 0, bot, 90, 'GND', str(n), hide=i > 0))
    return _symbol('Waveshare_ESP32-P4-Module', 'U', 'ESP32-P4-Module', 'mobot:Waveshare_ESP32-P4-Module',
                   'https://www.waveshare.com/wiki/ESP32-P4-Module',
                   'Waveshare ESP32-P4-Module: ESP32-P4NRW32 + 16MB flash + ESP32-C6 (Wi-Fi 6/BT via SDIO). '
                   '3.3 V only. * = VDD_IO_5 domain powered by ESP_LDO_VO4',
                   -22.86, top + 2.54, 22.86, bot + 2.54, L)


def servo_header():
    """4 servo channels x (SIG, V+, GND) on a 3x4 0.1" header. Pad = (ch-1)*3 + row (1 SIG, 2 V+, 3 GND)."""
    L = []
    names = ['ESC', 'STEER_F', 'STEER_R', 'SPARE']
    for ch in range(4):
        y = 3.81 - ch * 2.54
        L.append(_pin('passive', -17.78, y, 0, names[ch] + ' SIG', str(ch * 3 + 1)))
        L.append(_pin('passive', 17.78, y, 180, names[ch] + ' V+', str(ch * 3 + 2)))
        L.append(_pin('passive', 17.78, y - 12.7, 180, names[ch] + ' GND', str(ch * 3 + 3)))
    return _symbol('Servo_Header_3x04', 'J', 'Servo 3x4', 'mobot:PinHeader_3x04_P2.54mm_Servo', '',
                   'Four servo/ESC channels on one 3x4 0.1in header: columns = channels, rows = SIG / V+ / GND',
                   -15.24, 6.35, 15.24, -19.05, L)


def library():
    return [S('kicad_symbol_lib'), [S('version'), S('20251024')], [S('generator'), 'kicad_symbol_editor'],
            [S('generator_version'), '10.0'], sch16t(), p4_module(), servo_header()]


def write(path):
    with open(path, 'w') as f:
        f.write(dump(library()) + '\n')
