"""Generate the Ropener Bill of Materials (V2.0) as a branded .docx file.

Source data: BOM-V2.0.md (kept in sync by hand).
Run:  python make_bom.py
Output: "Ropener Bill of Materials.docx" in this folder.
"""

import os
import re

import docx
from docx import Document
from docx.shared import Pt, RGBColor, Inches
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.enum.table import WD_TABLE_ALIGNMENT
from docx.opc.constants import RELATIONSHIP_TYPE as RT
from docx.oxml.ns import qn
from docx.oxml import OxmlElement

# Valar Systems branding (matches the user guide).
ACCENT = RGBColor(0x00, 0x00, 0x00)   # brand black (headings)
ORANGE = RGBColor(0xF8, 0x90, 0x48)   # Valar accent orange
ORANGE_HEX = "F89048"
GREY = RGBColor(0x59, 0x59, 0x59)
LINK_BLUE = "0563C1"
MONO = "Consolas"
HERE = os.path.dirname(os.path.abspath(__file__))
LOGO = os.path.join(HERE, "valar-logo-black.png")

LINK_RE = re.compile(r"\[([^\]]+)\]\(([^)]+)\)")

doc = Document()
normal = doc.styles["Normal"]
normal.font.name = "Calibri"
normal.font.size = Pt(11)
for name, size in (("Heading 1", 16), ("Heading 2", 12.5)):
    st = doc.styles[name]
    st.font.color.rgb = ACCENT
    st.font.size = Pt(size)


def shade_cell(cell, hex_fill):
    tcPr = cell._tc.get_or_add_tcPr()
    shd = OxmlElement("w:shd")
    shd.set(qn("w:val"), "clear")
    shd.set(qn("w:color"), "auto")
    shd.set(qn("w:fill"), hex_fill)
    tcPr.append(shd)


def hrule(color_hex=ORANGE_HEX, size=18):
    p = doc.add_paragraph()
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    pPr = p._p.get_or_add_pPr()
    pbdr = OxmlElement("w:pBdr")
    bottom = OxmlElement("w:bottom")
    bottom.set(qn("w:val"), "single")
    bottom.set(qn("w:sz"), str(size))
    bottom.set(qn("w:space"), "1")
    bottom.set(qn("w:color"), color_hex)
    pbdr.append(bottom)
    pPr.append(pbdr)
    return p


def add_hyperlink(paragraph, url, text):
    r_id = paragraph.part.relate_to(url, RT.HYPERLINK, is_external=True)
    link = OxmlElement("w:hyperlink")
    link.set(qn("r:id"), r_id)
    run = OxmlElement("w:r")
    rPr = OxmlElement("w:rPr")
    color = OxmlElement("w:color")
    color.set(qn("w:val"), LINK_BLUE)
    rPr.append(color)
    u = OxmlElement("w:u")
    u.set(qn("w:val"), "single")
    rPr.append(u)
    run.append(rPr)
    t = OxmlElement("w:t")
    t.text = text
    t.set(qn("xml:space"), "preserve")
    run.append(t)
    link.append(run)
    paragraph._p.append(link)


def fill_desc(cell, text):
    """Write a description cell, turning [text](url) markdown into hyperlinks."""
    cell.text = ""
    p = cell.paragraphs[0]
    last = 0
    matched = False
    for m in LINK_RE.finditer(text):
        matched = True
        if m.start() > last:
            p.add_run(text[last:m.start()])
        add_hyperlink(p, m.group(2), m.group(1))
        last = m.end()
    if last < len(text):
        p.add_run(text[last:])
    if not matched and not text:
        p.add_run("")


def bom_table(rows, widths=(2.7, 0.7, 3.0)):
    t = doc.add_table(rows=1, cols=3)
    t.style = "Table Grid"
    t.alignment = WD_TABLE_ALIGNMENT.LEFT
    headers = ["Part", "Qty", "Description"]
    for i, h in enumerate(headers):
        c = t.rows[0].cells[i]
        c.text = ""
        shade_cell(c, ORANGE_HEX)
        run = c.paragraphs[0].add_run(h)
        run.bold = True
        run.font.color.rgb = RGBColor(0x00, 0x00, 0x00)
        if i == 1:
            c.paragraphs[0].alignment = WD_ALIGN_PARAGRAPH.CENTER
    for part, qty, desc in rows:
        cells = t.add_row().cells
        cells[0].text = ""
        cells[0].paragraphs[0].add_run(part).bold = True
        cells[1].text = str(qty)
        cells[1].paragraphs[0].alignment = WD_ALIGN_PARAGRAPH.CENTER
        fill_desc(cells[2], desc)
    for row in t.rows:
        for i, w in enumerate(widths):
            row.cells[i].width = Inches(w)
    doc.add_paragraph()
    return t


# ---------------------------------------------------------------------------
# Header / branding
# ---------------------------------------------------------------------------
logo_p = doc.add_paragraph()
logo_p.alignment = WD_ALIGN_PARAGRAPH.CENTER
if os.path.exists(LOGO):
    logo_p.add_run().add_picture(LOGO, width=Inches(3.2))

title = doc.add_paragraph()
title.alignment = WD_ALIGN_PARAGRAPH.CENTER
r = title.add_run("Ropener Curtain Controller")
r.bold = True
r.font.size = Pt(24)
r.font.color.rgb = ACCENT

sub = doc.add_paragraph()
sub.alignment = WD_ALIGN_PARAGRAPH.CENTER
r = sub.add_run("Bill of Materials")
r.bold = True
r.font.size = Pt(15)
r.font.color.rgb = ORANGE

ver = doc.add_paragraph()
ver.alignment = WD_ALIGN_PARAGRAPH.CENTER
r = ver.add_run("Version 2.0")
r.italic = True
r.font.color.rgb = GREY

hrule()
doc.add_paragraph()

intro = doc.add_paragraph()
intro.add_run(
    "Everything needed to build one Ropener. Quantities are per unit. Links in "
    "the Description column point to the custom PCB and suggested suppliers; "
    "equivalent parts may be substituted where noted."
)

# ---------------------------------------------------------------------------
# Body Assembly
# ---------------------------------------------------------------------------
doc.add_heading("Body Assembly", level=1)
bom_table([
    ("VAL3000 PCB", 1, "[Custom PCB designed for this project](https://github.com/Valar-Systems/VAL3000)"),
    ("NEMA 17 motor, 48 mm body", 1, "[Smaller length NEMA 17 will also work](https://amzn.to/4a2OnDi)"),
    ("12V / 2A+ power adapter w/ 5.5×2.1 mm plug", 1,
     "[Technically 5–29 V will work, but 12 V is best due to low speeds](https://amzn.to/3YauHG6)"),
    ("Extension cable", 1, "[Extends the length of the power adapter](https://amzn.to/4oHaTVT)"),
    ("M3 × 10 mm screw, self-tapping", 8, ""),
    ("M3 × 35 mm screw, button head", 3, ""),
    ("M3 square nut", 1, ""),
    ("M3 × 16 mm screw, button head", 2, ""),
    ("M3 × 10 mm screw, flat head", 2, ""),
    ("Gear", 1, "MK7"),
    ("Bearing", 2, "7 mm × 3 mm × 3 mm"),
])

# ---------------------------------------------------------------------------
# Curtain Assembly
# ---------------------------------------------------------------------------
doc.add_heading("Curtain Assembly", level=1)
bom_table([
    ("PTFE tubing, 3 mm ID × 5 mm OD", 1, "290 mm length required"),
    ("M3 × 10 mm screw, self-tapping", 11, ""),
    ("M3 square nut", 1, ""),
    ("M3 × 12 mm screw, flat head", 1, ""),
    ("Zip tie", 1, "Small, 2–3 mm wide"),
    ("V623ZZ pulley", 1, "3 × 12 × 4 mm size"),
    ("Rope", 1, "1.6 mm diameter"),
])

doc.add_heading("Mounting Hardware", level=2)
bom_table([
    ("Command strips", 2, "The easiest way to mount to any wall"),
    ('#17 × 1 1/4" wire nail', 3, "Use to nail to drywall only"),
])

# ---------------------------------------------------------------------------
# Assembly Tools Required
# ---------------------------------------------------------------------------
doc.add_heading("Assembly Tools Required", level=1)
bom_table([
    ("Razor", 1, "To cut the length of PTFE tubing"),
    ("Screwdriver, Phillips No. 1", 1, ""),
    ("Hex driver, 2 mm", 1, ""),
])

# ---------------------------------------------------------------------------
# Footer
# ---------------------------------------------------------------------------
footer = doc.sections[0].footer
fp = footer.paragraphs[0]
fp.text = ""
run = fp.add_run("Valar Systems")
run.bold = True
run.font.size = Pt(9)
run.font.color.rgb = ACCENT
bullet = fp.add_run("   •   ")
bullet.bold = True
bullet.font.size = Pt(9)
bullet.font.color.rgb = ORANGE
url = fp.add_run("valarsystems.com")
url.font.size = Pt(9)
url.font.color.rgb = GREY

out = os.path.join(HERE, "Ropener Bill of Materials.docx")
doc.save(out)
print("Saved:", out)
