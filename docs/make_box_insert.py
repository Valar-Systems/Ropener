"""Generate a 4x6 inch product-box insert card with a QR code to the wiki.

Run:  python make_box_insert.py
Output (in this folder):
  - "Ropener Box Insert.png"  (1200x1800 px, 300 DPI, ready for print services)
  - "Ropener Box Insert.pdf"  (same artwork, 4x6 in)
"""

import os
import segno
from PIL import Image, ImageDraw, ImageFont

# ---- config ---------------------------------------------------------------
WIKI_URL = "https://github.com/Valar-Systems/Ropener/wiki"
URL_LABEL = "github.com/Valar-Systems/Ropener/wiki"

HERE = os.path.dirname(os.path.abspath(__file__))
LOGO = os.path.join(HERE, "valar-logo-black.png")

# 4 x 6 in at 300 DPI (portrait)
DPI = 300
W, H = 4 * DPI, 6 * DPI          # 1200 x 1800

# Valar brand palette
ORANGE = (0xF8, 0x90, 0x48)
BLACK = (0x1A, 0x1A, 0x1A)
GREY = (0x59, 0x59, 0x59)
LGREY = (0xCF, 0xCF, 0xCF)
WHITE = (0xFF, 0xFF, 0xFF)

FONTS = os.path.join(os.environ.get("WINDIR", r"C:\Windows"), "Fonts")


def font(name, size):
    return ImageFont.truetype(os.path.join(FONTS, name), size)


BOLD = "segoeuib.ttf"
SEMI = "seguisb.ttf"
REG = "segoeui.ttf"
MONO = "consola.ttf"

img = Image.new("RGB", (W, H), WHITE)
d = ImageDraw.Draw(img)


def center(text, y, fnt, fill, tracking=0):
    """Draw horizontally-centered text with optional letter-spacing. Returns
    the y just below the drawn text."""
    if tracking:
        widths = [d.textlength(ch, font=fnt) for ch in text]
        total = sum(widths) + tracking * (len(text) - 1)
        x = (W - total) / 2
        for ch, w in zip(text, widths):
            d.text((x, y), ch, font=fnt, fill=fill)
            x += w + tracking
    else:
        w = d.textlength(text, font=fnt)
        d.text(((W - w) / 2, y), text, font=fnt, fill=fill)
    asc, desc = fnt.getmetrics()
    return y + asc + desc


# ---- decorative border ----------------------------------------------------
d.rounded_rectangle([46, 46, W - 46, H - 46], radius=64, outline=ORANGE, width=8)

# ---- logo -----------------------------------------------------------------
logo = Image.open(LOGO).convert("RGBA")
lw = 430
lh = round(lw * logo.height / logo.width)
logo = logo.resize((lw, lh), Image.LANCZOS)
img.paste(logo, ((W - lw) // 2, 150), logo)

# ---- title block ----------------------------------------------------------
y = 150 + lh + 70
y = center("Ropener", y, font(BOLD, 120), BLACK)
y = center("Automated Curtain Opener", y + 8, font(REG, 42), GREY)

# short orange divider
d.line([(W / 2 - 110, y + 40), (W / 2 + 110, y + 40)], fill=ORANGE, width=6)

# ---- scan prompt ----------------------------------------------------------
center("SCAN TO SET UP YOUR ROPENER", y + 80, font(SEMI, 38), ORANGE, tracking=4)

# ---- QR code --------------------------------------------------------------
qr = segno.make(WIKI_URL, error="m")
matrix = list(qr.matrix)
n = len(matrix)
quiet = 4                                   # quiet-zone modules each side
target = 600                                # approx QR pixel size
mod = target // (n + quiet * 2)
qr_px = mod * (n + quiet * 2)

qr_x = (W - qr_px) // 2
qr_y = y + 170

# soft framing card behind the QR
pad = 40
d.rounded_rectangle(
    [qr_x - pad, qr_y - pad, qr_x + qr_px + pad, qr_y + qr_px + pad],
    radius=36, fill=WHITE, outline=LGREY, width=3,
)

for r, row in enumerate(matrix):
    for c, val in enumerate(row):
        if val:
            x0 = qr_x + (c + quiet) * mod
            y0 = qr_y + (r + quiet) * mod
            d.rectangle([x0, y0, x0 + mod - 1, y0 + mod - 1], fill=BLACK)

y = qr_y + qr_px + pad

# ---- url + value line -----------------------------------------------------
y = center(URL_LABEL, y + 46, font(MONO, 30), GREY)
center("Wi-Fi setup  •  Daily schedule  •  Troubleshooting",
       y + 22, font(REG, 32), BLACK)

# ---- footer ---------------------------------------------------------------
center("Need help? Open an issue on GitHub", H - 168, font(REG, 30), GREY)
center("valarsystems.com", H - 124, font(SEMI, 32), ORANGE)

# ---- save -----------------------------------------------------------------
png_out = os.path.join(HERE, "Ropener Box Insert.png")
pdf_out = os.path.join(HERE, "Ropener Box Insert.pdf")
img.save(png_out, dpi=(DPI, DPI))
img.save(pdf_out, "PDF", resolution=DPI)
print("Saved:", png_out)
print("Saved:", pdf_out)
