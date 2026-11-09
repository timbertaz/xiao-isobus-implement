#!/usr/bin/env python3
"""Builds the implement's ISOBUS screen (object pool, ISO 11783-6) for the Norén demo.

    python tools/make_pool.py

Writes:
    src/object_pool/object_pool.iop   the pool the XIAO uploads to the terminal
    src/noren_pool.hpp                object IDs for main.cpp

The screen (480 x 480 data mask):
    header   Norén logo + data source ("TRACTOR" / "SIMULATED")
    speed    big km/h value
    hitch    rear hitch position in % with a bar
    PTO      rear PTO speed in rpm
    lights   three buttons for the implement's own lights
Soft key: "ALL OFF" (lights).

Needs Pillow for the logo (pip install pillow).
"""

import os
import struct

from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)

# ---------------------------------------------------------------- VT colour palette
# ISO 11783-6: 0-15 standard colours, 16-231 = 6x6x6 cube (00,33,66,99,CC,FF)
STANDARD = [(0, 0, 0), (255, 255, 255), (0, 153, 0), (0, 153, 153), (153, 0, 0), (153, 0, 153), (153, 153, 0),
            (204, 204, 204), (153, 153, 153), (0, 0, 255), (0, 255, 0), (0, 255, 255), (255, 0, 0), (255, 0, 255),
            (255, 255, 0), (0, 0, 153)]
LEVELS = [0, 51, 102, 153, 204, 255]
PALETTE = STANDARD + [(LEVELS[r], LEVELS[g], LEVELS[b]) for r in range(6) for g in range(6) for b in range(6)]


def cube(r, g, b):
    """Colour index from 0-5 levels per channel (0x00, 0x33 ... 0xFF)."""
    return 16 + r * 36 + g * 6 + b


BLACK = 0
WHITE = 1
GREY = 8                  # 999999
PANEL = cube(1, 1, 1)     # 333333
DARK = cube(2, 2, 2)      # 666666
YELLOW = cube(5, 4, 0)    # FFCC00, JCB-ish yellow
GREEN = cube(1, 4, 2)     # 33CC66


def nearest(rgb):
    best, bestDist = 0, 1 << 30
    for i, p in enumerate(PALETTE):
        d = sum((a - b) ** 2 for a, b in zip(rgb, p))
        if d < bestDist:
            best, bestDist = i, d
    return best


# ---------------------------------------------------------------- object IDs
ID = {
    'WORKING_SET': 0,
    'MAIN_MASK': 1000,
    'SOFT_KEYS': 2000,
    'KEY_ALL_OFF': 3000,
    'BTN_WORK': 6001, 'BTN_BEACON': 6002, 'BTN_REAR': 6003,
    'STR_SOURCE': 11001,
    'STR_WORK_STATE': 11011, 'STR_BEACON_STATE': 11012, 'STR_REAR_STATE': 11013,
    'NUM_SPEED': 12001, 'NUM_HITCH': 12002, 'NUM_PTO': 12003,
    'BAR_HITCH': 18001,
    'LOGO': 20000,
    'VAR_SPEED': 21001, 'VAR_HITCH': 21002, 'VAR_PTO': 21003,
}
_next_free = [40000]


def new_id():
    _next_free[0] += 1
    return _next_free[0]


# ---------------------------------------------------------------- object encoders
def u16(v):
    return struct.pack('<H', v)


def u32(v):
    return struct.pack('<I', v & 0xFFFFFFFF)


def children6(kids):
    return b''.join(u16(i) + struct.pack('<hh', x, y) for i, x, y in kids)


objects = []


def add(data):
    objects.append(data)


def working_set(oid, bg, active_mask, kids, languages=('en',)):
    add(u16(oid) + bytes([0, bg, 1]) + u16(active_mask) + bytes([len(kids), 0, len(languages)]) + children6(kids) +
        b''.join(l.encode('ascii') for l in languages))


def data_mask(oid, bg, soft_key_mask, kids):
    add(u16(oid) + bytes([1, bg]) + u16(soft_key_mask) + bytes([len(kids), 0]) + children6(kids))


def soft_key_mask(oid, bg, keys):
    add(u16(oid) + bytes([4, bg, len(keys), 0]) + b''.join(u16(k) for k in keys))


def key(oid, bg, key_code, kids):
    add(u16(oid) + bytes([5, bg, key_code, len(kids), 0]) + children6(kids))


def button(oid, w, h, bg, border, key_code, kids, options=0):
    add(u16(oid) + bytes([6]) + u16(w) + u16(h) + bytes([bg, border, key_code, options, len(kids), 0]) + children6(kids))


def output_string(oid, w, h, bg, font, text, length=None, transparent=True, justify=0):
    length = length or len(text)
    value = text.encode('latin-1').ljust(length, b' ')[:length]
    options = 0x01 if transparent else 0x00
    add(u16(oid) + bytes([11]) + u16(w) + u16(h) + bytes([bg]) + u16(font) + bytes([options]) + u16(0xFFFF) +
        bytes([justify]) + u16(length) + value + bytes([0]))


def output_number(oid, w, h, bg, font, variable, scale=1.0, decimals=0, justify=0, transparent=True):
    options = 0x01 if transparent else 0x00
    add(u16(oid) + bytes([12]) + u16(w) + u16(h) + bytes([bg]) + u16(font) + bytes([options]) + u16(variable) +
        u32(0) + struct.pack('<i', 0) + struct.pack('<f', scale) + bytes([decimals, 0, justify, 0]))


def rectangle(oid, w, h, line, fill, suppress=0):
    add(u16(oid) + bytes([14]) + u16(line) + u16(w) + u16(h) + bytes([suppress]) + u16(fill) + bytes([0]))


def linear_bar(oid, w, h, colour, variable, minimum, maximum, options):
    add(u16(oid) + bytes([18]) + u16(w) + u16(h) + bytes([colour, colour, options, 0]) + u16(minimum) + u16(maximum) +
        u16(variable) + u16(0) + u16(0xFFFF) + u16(0) + bytes([0]))


def number_variable(oid, value=0):
    add(u16(oid) + bytes([21]) + u32(value))


def font_attributes(oid, colour, size):
    add(u16(oid) + bytes([23, colour, size, 0, 0, 0]))  # type 0 = ISO 8859-1, style normal


def line_attributes(oid, colour, width):
    add(u16(oid) + bytes([24, colour, width]) + u16(0xFFFF) + bytes([0]))


def fill_attributes(oid, colour):
    add(u16(oid) + bytes([25, 2, colour]) + u16(0xFFFF) + bytes([0]))  # 2 = fill with fill colour


def picture(oid, pixels, w, h):
    """8-bit colour picture, run-length encoded (count, colour) pairs."""
    rle = bytearray()
    i = 0
    while i < len(pixels):
        run = 1
        while (i + run < len(pixels)) and (pixels[i + run] == pixels[i]) and (run < 255):
            run += 1
        rle += bytes([run, pixels[i]])
        i += run
    add(u16(oid) + bytes([20]) + u16(w) + u16(w) + u16(h) + bytes([2, 0x04, 0]) + u32(len(rle)) + bytes([0]) + bytes(rle))


# ---------------------------------------------------------------- logo
def make_logo(width, height, background):
    """'Norén' wordmark in white with a yellow bar, on the header colour, quantised to the VT palette."""
    scale = 4  # draw big, then shrink: smooth edges
    img = Image.new('RGB', (width * scale, height * scale), PALETTE[background])
    draw = ImageDraw.Draw(img)
    font = ImageFont.truetype('/usr/share/fonts/truetype/google-fonts/Poppins-Bold.ttf', int(height * scale * 0.82))
    text = 'Norén'
    box = draw.textbbox((0, 0), text, font=font)
    tx = -box[0] + int(4 * scale)
    ty = (height * scale - (box[3] - box[1])) // 2 - box[1] - int(3 * scale)
    draw.text((tx, ty), text, font=font, fill=(255, 255, 255))
    bar_y = height * scale - int(7 * scale)
    draw.rectangle([tx, bar_y, tx + (box[2] - box[0]), bar_y + int(4 * scale)], fill=PALETTE[YELLOW])
    img = img.resize((width, height), Image.LANCZOS)
    cache = {}
    pixels = []
    raw = img.tobytes()
    for p in (tuple(raw[i:i + 3]) for i in range(0, len(raw), 3)):
        if p not in cache:
            cache[p] = nearest(p)
        pixels.append(cache[p])
    return pixels


# ---------------------------------------------------------------- the screen
def build():
    f_small = new_id()   # 8x12 grey labels
    f_label = new_id()   # 12x16 black, button labels
    f_status = new_id()  # 12x16 yellow, data source
    f_unit = new_id()    # 16x24 grey units
    f_mid = new_id()     # 32x48 white values
    f_big = new_id()     # 48x64 white speed
    f_key = new_id()     # 8x12 white soft key
    line_none = new_id()
    line_border = new_id()
    fill_panel = new_id()

    font_attributes(f_small, GREY, 2)
    font_attributes(f_label, BLACK, 3)
    font_attributes(f_status, YELLOW, 3)
    font_attributes(f_unit, GREY, 5)
    font_attributes(f_mid, WHITE, 8)
    font_attributes(f_big, WHITE, 9)
    font_attributes(f_key, WHITE, 2)
    line_attributes(line_none, PANEL, 0)
    line_attributes(line_border, DARK, 1)
    fill_attributes(fill_panel, PANEL)

    # Variables the XIAO updates
    number_variable(ID['VAR_SPEED'])  # km/h x 10
    number_variable(ID['VAR_HITCH'])  # %
    number_variable(ID['VAR_PTO'])    # rpm

    kids = []

    def place(oid, x, y):
        kids.append((oid, x, y))

    # Header with logo and data source
    header = new_id()
    rectangle(header, 480, 76, line_none, fill_panel)
    place(header, 0, 0)
    logo_w, logo_h = 210, 64
    picture(ID['LOGO'], make_logo(logo_w, logo_h, PANEL), logo_w, logo_h)
    place(ID['LOGO'], 14, 6)
    output_string(ID['STR_SOURCE'], 170, 20, PANEL, f_status, 'WAITING', length=9, justify=2)
    place(ID['STR_SOURCE'], 296, 18)
    caption = new_id()
    output_string(caption, 170, 14, PANEL, f_small, 'DATA SOURCE', justify=2)
    place(caption, 296, 44)

    # Tiles
    def tile(x, y, w, h, title):
        r = new_id()
        rectangle(r, w, h, line_border, fill_panel)
        place(r, x, y)
        t = new_id()
        output_string(t, w - 24, 14, PANEL, f_small, title)
        place(t, x + 12, y + 10)

    tile(16, 90, 448, 150, 'GROUND SPEED')
    output_number(ID['NUM_SPEED'], 200, 64, PANEL, f_big, ID['VAR_SPEED'], scale=0.1, decimals=1, justify=2)
    place(ID['NUM_SPEED'], 30, 130)
    unit = new_id()
    output_string(unit, 90, 24, PANEL, f_unit, 'km/h')
    place(unit, 240, 160)

    tile(16, 254, 216, 120, 'REAR HITCH')
    output_number(ID['NUM_HITCH'], 100, 48, PANEL, f_mid, ID['VAR_HITCH'], justify=2)
    place(ID['NUM_HITCH'], 20, 278)
    pct = new_id()
    output_string(pct, 40, 24, PANEL, f_unit, '%')
    place(pct, 126, 294)
    linear_bar(ID['BAR_HITCH'], 192, 16, YELLOW, ID['VAR_HITCH'], 0, 100, 0x31)  # border, horizontal, grows right
    place(ID['BAR_HITCH'], 28, 342)

    tile(248, 254, 216, 120, 'REAR PTO')
    output_number(ID['NUM_PTO'], 130, 48, PANEL, f_mid, ID['VAR_PTO'], justify=2)
    place(ID['NUM_PTO'], 252, 278)
    rpm = new_id()
    output_string(rpm, 60, 24, PANEL, f_unit, 'rpm')
    place(rpm, 388, 294)

    # Light buttons (momentary; the XIAO toggles the light and recolours the button)
    lights_title = new_id()
    output_string(lights_title, 300, 14, BLACK, f_small, 'IMPLEMENT LIGHTS')
    place(lights_title, 16, 386)
    for i, (name, bid, sid) in enumerate([('WORK', 'BTN_WORK', 'STR_WORK_STATE'),
                                          ('BEACON', 'BTN_BEACON', 'STR_BEACON_STATE'),
                                          ('REAR', 'BTN_REAR', 'STR_REAR_STATE')]):
        name_id = new_id()
        output_string(name_id, 130, 18, GREY, f_label, name, justify=1)
        output_string(ID[sid], 130, 18, GREY, f_label, 'OFF', length=3, justify=1)
        button(ID[bid], 144, 68, GREY, BLACK, i + 1, [(name_id, 0, 14), (ID[sid], 0, 36)])
        place(ID[bid], 16 + i * 152, 404)

    data_mask(ID['MAIN_MASK'], BLACK, ID['SOFT_KEYS'], kids)

    # Soft key: all lights off
    key_text = new_id()
    output_string(key_text, 56, 32, DARK, f_key, 'ALL OFF', length=7, justify=0x05)
    key(ID['KEY_ALL_OFF'], DARK, 10, [(key_text, 2, 14)])
    soft_key_mask(ID['SOFT_KEYS'], BLACK, [ID['KEY_ALL_OFF']])

    working_set(ID['WORKING_SET'], BLACK, ID['MAIN_MASK'], [(ID['LOGO'], 0, 0)])


def main():
    build()
    # The working set must come first
    ws = [o for o in objects if o[2] == 0]
    rest = [o for o in objects if o[2] != 0]
    pool = b''.join(ws + rest)

    out = os.path.join(ROOT, 'src', 'object_pool', 'object_pool.iop')
    with open(out, 'wb') as f:
        f.write(pool)

    header = os.path.join(ROOT, 'src', 'noren_pool.hpp')
    with open(header, 'w') as f:
        f.write('// Generated by tools/make_pool.py - do not edit, run the script instead\n#pragma once\n#include <cstdint>\n\n')
        f.write('namespace pool\n{\n')
        for name, value in ID.items():
            f.write('\tconstexpr std::uint16_t %s = %d;\n' % (name, value))
        f.write('\n\tconstexpr std::uint8_t COLOUR_LIGHT_OFF = %d;\n' % GREY)
        f.write('\tconstexpr std::uint8_t COLOUR_LIGHT_ON = %d;\n' % YELLOW)
        f.write('\tconstexpr std::uint8_t COLOUR_SOURCE_TRACTOR = %d;\n' % GREEN)
        f.write('}\n')
    print('%s: %d objects, %d bytes' % (out, len(objects), len(pool)))


if __name__ == '__main__':
    main()
