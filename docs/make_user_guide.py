"""Generate the Ropener Curtain Controller user guide as a .docx file.

Run:  python make_user_guide.py
Output: "Ropener User Guide.docx" in this folder.
"""

import os

from docx import Document
from docx.shared import Pt, RGBColor, Inches
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.enum.table import WD_TABLE_ALIGNMENT
from docx.oxml.ns import qn
from docx.oxml import OxmlElement

# Valar Systems branding (from the logo): black "VALAR" wordmark with an orange
# "systems" script. Headings stay black for readability; orange (#F89048) is
# the accent — divider rule, table header rows, and footer.
ACCENT = RGBColor(0x00, 0x00, 0x00)   # brand black (headings)
ORANGE = RGBColor(0xF8, 0x90, 0x48)   # Valar accent orange (sampled from logo)
ORANGE_HEX = "F89048"
GREY = RGBColor(0x59, 0x59, 0x59)     # muted grey for sub-text
MONO = "Consolas"
HERE = os.path.dirname(os.path.abspath(__file__))
LOGO = os.path.join(HERE, "valar-logo-black.png")  # transparent bg, sits on white

doc = Document()


def shade_cell(cell, hex_fill):
    """Fill a table cell with a solid background color."""
    tcPr = cell._tc.get_or_add_tcPr()
    shd = OxmlElement("w:shd")
    shd.set(qn("w:val"), "clear")
    shd.set(qn("w:color"), "auto")
    shd.set(qn("w:fill"), hex_fill)
    tcPr.append(shd)


def hrule(color_hex=ORANGE_HEX, size=18):
    """Add a thin colored horizontal divider rule."""
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

# ---- base styles -----------------------------------------------------------
normal = doc.styles["Normal"]
normal.font.name = "Calibri"
normal.font.size = Pt(11)

for name, size in (("Heading 1", 16), ("Heading 2", 13), ("Heading 3", 11.5)):
    st = doc.styles[name]
    st.font.color.rgb = ACCENT
    st.font.size = Pt(size)


def code(text):
    """Add a paragraph of monospaced text (for POSIX strings / commands)."""
    p = doc.add_paragraph()
    run = p.add_run(text)
    run.font.name = MONO
    run.font.size = Pt(10)
    return p


def callout(title, body, fill="FCE3CF"):
    """A shaded attention box (light orange) with a bold title and body text."""
    t = doc.add_table(rows=1, cols=1)
    t.style = "Table Grid"
    cell = t.rows[0].cells[0]
    shade_cell(cell, fill)
    cell.text = ""
    tp = cell.paragraphs[0]
    tr = tp.add_run(title)
    tr.bold = True
    tr.font.color.rgb = RGBColor(0x9C, 0x4A, 0x00)   # dark orange for contrast
    bp = cell.add_paragraph()
    bp.add_run(body)
    doc.add_paragraph()


def bullets(items, style="List Bullet"):
    for it in items:
        if isinstance(it, tuple):
            p = doc.add_paragraph(style=style)
            r = p.add_run(it[0])
            r.bold = True
            p.add_run(it[1])
        else:
            doc.add_paragraph(it, style=style)


def mono_table(headers, rows, widths=None):
    t = doc.add_table(rows=1, cols=len(headers))
    t.style = "Table Grid"
    t.alignment = WD_TABLE_ALIGNMENT.LEFT
    hdr = t.rows[0].cells
    for i, h in enumerate(headers):
        hdr[i].text = ""
        shade_cell(hdr[i], ORANGE_HEX)          # brand-orange header row
        run = hdr[i].paragraphs[0].add_run(h)
        run.bold = True
        run.font.color.rgb = RGBColor(0x00, 0x00, 0x00)
    for row in rows:
        cells = t.add_row().cells
        for i, val in enumerate(row):
            cells[i].text = ""
            run = cells[i].paragraphs[0].add_run(val)
            # monospace any cell that looks like a POSIX string / value
            if any(ch in val for ch in (",", "<")) or val[:3] in (
                "EST", "CST", "MST", "PST", "AKS", "HST", "SST", "ChS", "AST", "UTC",
                "WET", "GMT", "CET", "EET", "MSK",
            ):
                run.font.name = MONO
                run.font.size = Pt(9.5)
    if widths:
        for row in t.rows:
            for i, w in enumerate(widths):
                row.cells[i].width = Inches(w)
    return t


# ============================================================================
# TITLE / BRANDING
# ============================================================================
logo_p = doc.add_paragraph()
logo_p.alignment = WD_ALIGN_PARAGRAPH.CENTER
if os.path.exists(LOGO):
    logo_p.add_run().add_picture(LOGO, width=Inches(3.2))

title = doc.add_paragraph()
title.alignment = WD_ALIGN_PARAGRAPH.CENTER
r = title.add_run("Ropener Curtain Controller")
r.bold = True
r.font.size = Pt(26)
r.font.color.rgb = ACCENT

sub = doc.add_paragraph()
sub.alignment = WD_ALIGN_PARAGRAPH.CENTER
r = sub.add_run("User Guide")
r.bold = True
r.font.size = Pt(15)
r.font.color.rgb = ORANGE

ver = doc.add_paragraph()
ver.alignment = WD_ALIGN_PARAGRAPH.CENTER
r = ver.add_run("Firmware 2.4.0")
r.italic = True
r.font.color.rgb = GREY

hrule()              # orange divider under the title block
doc.add_paragraph()

# Branded footer on every page: company name + accent bullet + site.
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

intro = doc.add_paragraph()
intro.add_run(
    "Ropener is a Wi-Fi connected curtain/cover controller. It pulls a poly "
    "rope with a small stepper motor to open and close your curtains. You can "
    "control it from any web browser on your network — no Home Assistant or "
    "cloud account required — or pair it with Home Assistant if you use one. "
    "It supports a daily open/close schedule (by fixed clock times or "
    "sunrise/sunset), automatic position calibration, and several tuning options."
)

# ============================================================================
# 1. AT A GLANCE
# ============================================================================
doc.add_heading("1. At a Glance", level=1)
bullets([
    ("Control from a browser: ", "open the device's web page on your phone or computer."),
    ("Three ways to operate: ", "the web page, the physical buttons on the device, or Home Assistant."),
    ("Open / close / partial: ", "drive the curtain fully open, fully closed, or to any position in between."),
    ("Daily schedule: ", "open and close automatically every day — at fixed clock times, or following sunrise and sunset."),
    ("Self-calibrating: ", "the device finds the fully-closed position by itself (“homing”)."),
])

# ============================================================================
# 2. FIRST-TIME SETUP: CONNECT TO WI-FI
# ============================================================================
doc.add_heading("2. First-Time Setup: Connect to Wi-Fi", level=1)
doc.add_paragraph(
    "The device ships without any saved Wi-Fi network, so the first thing to do "
    "is tell it which network to join. There are two ways to do this."
)

doc.add_heading("Option A — The setup hotspot (recommended)", level=2)
bullets([
    "Power on the device and wait about a minute.",
    "On your phone or laptop, open the Wi-Fi network list. You will see a network "
    "named “ropener-XXXXXX”, where XXXXXX is a unique code for your device.",
    "Write down or take a photo of this full name now — you will need the same "
    "“ropener-XXXXXX” to open the device in your browser later (see Section 3). "
    "The hotspot disappears once setup is done, so note it before continuing.",
    "Connect to it. A setup page should open automatically (this is the “captive "
    "portal”). If it does not, open a browser and go to http://192.168.4.1.",
    "Choose your home Wi-Fi network from the list, enter its password, and save.",
    "The device will reboot and join your network. The setup hotspot disappears once "
    "it connects successfully.",
])
tip = doc.add_paragraph()
tip.add_run("Remember: ").bold = True
tip.add_run(
    "the “ropener-XXXXXX” code is also your device's web address — you'll type "
    "http://ropener-XXXXXX.local into a browser to control it. Keep the code "
    "handy.")

doc.add_heading("Option B — Over USB (Improv)", level=2)
doc.add_paragraph(
    "If the device is plugged into a computer over USB, you can provision it from "
    "a web tool that supports “Improv” (for example, web.esphome.io). Open the "
    "tool, connect to the device's serial port, and enter your Wi-Fi details when "
    "prompted."
)

note = doc.add_paragraph()
note.add_run("Note: ").bold = True
note.add_run(
    "Your Wi-Fi details are stored on the device and kept across reboots and "
    "firmware updates. You only need to do this once (unless you reset Wi-Fi — "
    "see Section 9)."
)

# ============================================================================
# 3. OPEN THE WEB CONTROL PANEL
# ============================================================================
doc.add_heading("3. Open the Web Control Panel", level=1)
doc.add_paragraph(
    "Once the device is on your network, open a web browser on the same network "
    "and go to:"
)
code("http://ropener-XXXXXX.local")
doc.add_paragraph(
    "Replace XXXXXX with your device's code (the same code shown in the setup "
    "hotspot name). The page lists every control and reading described in this "
    "guide. Bookmark it for easy access."
)
tip = doc.add_paragraph()
tip.add_run("Tip: ").bold = True
tip.add_run(
    "If the .local address does not load, your network may not support it. In that "
    "case, find the device's IP address in your router and use http://<that-ip> "
    "instead."
)

# ============================================================================
# 4. EVERYDAY USE
# ============================================================================
doc.add_heading("4. Everyday Use", level=1)

doc.add_heading("From the web page or Home Assistant", level=2)
bullets([
    ("Open: ", "raises the cover to the fully-open position."),
    ("Close: ", "lowers the cover to the fully-closed position."),
    ("Set a position: ", "drag the cover slider to any point from 0% (closed) to 100% (open)."),
    ("Stop: ", "halts movement immediately and leaves the cover where it is."),
])

doc.add_heading("Using the physical buttons", level=2)
doc.add_paragraph(
    "The device has three buttons. While the cover is moving, a press of Button 1 "
    "or Button 2 simply stops it."
)
mono_table(
    ["Button", "Action", "What it does"],
    [
        ["Button 1", "Short press (about 1 sec)", "Closes the cover (or stops it if moving)."],
        ["Button 1", "Long press (3–10 sec)", "Starts homing / calibration (see Section 5)."],
        ["Button 2", "Press and release", "Opens the cover (or stops it if moving)."],
        ["Button 3", "Long press (3–10 sec)", "Clears saved Wi-Fi and reboots into setup (see Section 9)."],
    ],
    widths=[1.0, 2.0, 3.4],
)

# ============================================================================
# 5. HOMING
# ============================================================================
doc.add_heading("5. Homing (Calibrating the Closed Position)", level=1)

callout(
    "Important — you must stop homing manually (automatic stop is disabled)",
    "The sensor that normally detects the closed end (called “StallGuard”) is not "
    "working in this version of the firmware. During homing the motor will NOT "
    "stop by itself. You must watch the curtain and press Stop the moment it "
    "reaches the fully-closed position. If you do not stop it, the motor will keep "
    "running and strain against the stop. Always stay with the device while homing.",
)

doc.add_paragraph(
    "“Homing” teaches the device where the fully-closed position is. The motor "
    "drives the curtain toward the closed end; when it arrives, you stop it, and "
    "the device marks that spot as 0% (closed). Everything else — the open "
    "position, percentages, and the schedule — is measured from there."
)
doc.add_paragraph("Run homing when:")
bullets([
    "You first install the device.",
    "The reported position no longer matches reality (for example after the rope "
    "slipped or was pulled by hand).",
])
doc.add_heading("How to home (stop it yourself)", level=2)
bullets([
    "Start homing: press and hold Button 1 for 3 to 10 seconds, or click the "
    "“Start-Stop Homing” button on the web page.",
    "Watch the curtain drive toward the closed end.",
    "The instant it reaches the fully-closed position, press Stop — press Button 1 "
    "once, or click “Start-Stop Homing” again. This marks that spot as 0% (closed) "
    "and ends homing.",
], style="List Number")
doc.add_paragraph(
    "The “State” reading on the web page shows HOMING while homing is running and "
    "returns to IDLE once you press Stop."
)

# ============================================================================
# 6. DAILY SCHEDULE
# ============================================================================
doc.add_heading("6. Set Up the Daily Schedule", level=1)
doc.add_paragraph(
    "The device can open and close the curtain automatically every day. There are "
    "two scheduling modes — pick the one you prefer:"
)
bullets([
    ("Fixed times — ", "open and close at clock times you choose (e.g. 07:00 and 21:00)."),
    ("Sunrise / sunset — ", "open at sunrise and close at sunset, so the curtain "
     "follows the daylight as it shifts through the seasons."),
])
doc.add_paragraph(
    "“Schedule Enabled” is the master on/off for whichever mode is active. The "
    "“Sun Schedule” switch chooses between the two: OFF uses the fixed times, ON "
    "uses sunrise/sunset. Both modes run on local time, so set your Timezone first "
    "(Section 7)."
)

doc.add_heading("Fixed times (default)", level=2)
mono_table(
    ["Control", "What to set"],
    [
        ["Open Time", "The time of day to open the cover (e.g. 07:00)."],
        ["Close Time", "The time of day to close the cover (e.g. 21:00)."],
        ["Sun Schedule", "Leave this OFF to use fixed times."],
        ["Schedule Enabled", "Turn this ON to run the schedule, OFF to pause it."],
        ["Timezone", "Your local timezone, so the times mean what you expect (see Section 7)."],
    ],
    widths=[1.7, 4.7],
)
doc.add_paragraph("Steps:")
bullets([
    "Set your Timezone first (Section 7) — the open/close times are in local time.",
    "Set Open Time and Close Time.",
    "Make sure Sun Schedule is OFF.",
    "Turn Schedule Enabled ON.",
], style="List Number")

doc.add_heading("Sunrise / sunset (Sun Schedule)", level=2)
doc.add_paragraph(
    "Turn “Sun Schedule” ON to follow the sun instead of the clock. The curtain "
    "opens at sunrise and closes at sunset for your location, recalculated every "
    "day so it tracks the changing seasons automatically — no need to adjust times "
    "by hand."
)
doc.add_paragraph(
    "For this to be accurate, the device needs to know where it is. Enter your "
    "Latitude and Longitude in decimal degrees (north and east are positive; south "
    "and west are negative). You can copy these from any maps app — for example, "
    "New York is about Latitude 40.71, Longitude -74.01."
)
mono_table(
    ["Control", "What to set"],
    [
        ["Sun Schedule", "Turn this ON to use sunrise/sunset."],
        ["Latitude", "Your location's latitude in decimal degrees (e.g. 40.71)."],
        ["Longitude", "Your location's longitude in decimal degrees (e.g. -74.01)."],
        ["Sun Open Offset", "Minutes to shift the morning open relative to sunrise (see below)."],
        ["Sun Close Offset", "Minutes to shift the evening close relative to sunset (see below)."],
        ["Schedule Enabled", "Turn this ON to run the schedule."],
        ["Timezone", "Your local timezone (see Section 7)."],
    ],
    widths=[1.7, 4.7],
)
doc.add_paragraph("Offsets let you open or close a little before or after the sun, in minutes:")
bullets([
    "A positive offset is after the event; a negative offset is before it.",
    "To open 30 minutes after sunrise, set Sun Open Offset to 30.",
    "To close 30 minutes before sunset, set Sun Close Offset to -30.",
    "Leave both at 0 to open exactly at sunrise and close exactly at sunset.",
])
doc.add_paragraph("Steps:")
bullets([
    "Set your Timezone (Section 7).",
    "Enter your Latitude and Longitude.",
    "Optionally set the open/close offsets.",
    "Turn Sun Schedule ON.",
    "Turn Schedule Enabled ON.",
], style="List Number")

note = doc.add_paragraph()
note.add_run("Notes: ").bold = True
note.add_run(
    "Both schedules need an internet connection to keep accurate time. Scheduled "
    "moves are skipped while the device is homing, and every schedule setting is "
    "remembered across reboots. With Sun Schedule ON the fixed Open Time / Close "
    "Time are ignored. If you leave Latitude and Longitude at 0, sunrise/sunset "
    "will not match your location — set them before relying on the sun schedule."
)

# ============================================================================
# 7. TIMEZONE
# ============================================================================
doc.add_heading("7. Setting Your Timezone", level=1)
doc.add_paragraph(
    "The device keeps time using the internet, which provides time in UTC. To make "
    "the schedule fire at the correct local time, enter your timezone in the "
    "“Timezone” box using a POSIX timezone string. Copy the value for your region "
    "from the table in Section 8 and paste it into the Timezone box, then save."
)
p = doc.add_paragraph()
p.add_run("Example: ").bold = True
p.add_run("for U.S. Eastern time with daylight saving, enter ")
r = p.add_run("EST5EDT,M3.2.0,M11.1.0")
r.font.name = MONO
p.add_run(".")

warn = doc.add_paragraph()
warn.add_run("Important about the number sign: ").bold = True
warn.add_run(
    "In a POSIX timezone string the number is the hours WEST of UTC, so it looks "
    "“backwards” compared to the usual UTC offset. U.S. Eastern (UTC−5) is written "
    "as 5, Pacific (UTC−8) as 8, and so on. The table in Section 8 already has this "
    "right — just copy it exactly, including capitalization and punctuation."
)
doc.add_paragraph(
    "The trailing parts (for example “,M3.2.0,M11.1.0”) are the daylight-saving "
    "rules. The U.S. switches on the 2nd Sunday of March and the 1st Sunday of "
    "November; the European Union switches on the last Sunday of March and the last "
    "Sunday of October. The tables in Section 8 already encode the correct rule for "
    "each region — just copy the value exactly. If your area does not observe "
    "daylight saving time, use the “without DST” value, which has no trailing rules."
)

# ============================================================================
# 8. US TIMEZONE TABLE
# ============================================================================
doc.add_heading("8. Timezone POSIX Strings", level=1)
doc.add_paragraph(
    "Find your region below and copy the matching string into the Timezone box. "
    "Use the “With DST” column if your area changes its clocks twice a year; use "
    "“Without DST (standard all year)” if it does not."
)

doc.add_heading("United States", level=2)
tz_rows = [
    ["Eastern", "New York, Florida, Ohio, most of the East Coast",
     "EST5EDT,M3.2.0,M11.1.0", "EST5"],
    ["Central", "Texas, Illinois, Tennessee (western), most of the Midwest",
     "CST6CDT,M3.2.0,M11.1.0", "CST6"],
    ["Mountain", "Colorado, Utah, Montana, New Mexico",
     "MST7MDT,M3.2.0,M11.1.0", "MST7"],
    ["Mountain (Arizona)", "Most of Arizona — no daylight saving",
     "(not used)", "MST7"],
    ["Pacific", "California, Washington, Oregon, Nevada",
     "PST8PDT,M3.2.0,M11.1.0", "PST8"],
    ["Alaska", "Most of Alaska",
     "AKST9AKDT,M3.2.0,M11.1.0", "AKST9"],
    ["Hawaii–Aleutian (Aleutian Is.)", "Western Aleutian Islands, AK — observes DST",
     "HST10HDT,M3.2.0,M11.1.0", "HST10"],
    ["Hawaii", "State of Hawaii — no daylight saving",
     "(not used)", "HST10"],
    ["Atlantic", "Puerto Rico, U.S. Virgin Islands — no daylight saving",
     "(not used)", "AST4"],
    ["Samoa", "American Samoa — no daylight saving",
     "(not used)", "SST11"],
    ["Chamorro", "Guam, Northern Mariana Islands — no daylight saving",
     "(not used)", "ChST-10"],
    ["Wake Island", "Wake Island — no daylight saving",
     "(not used)", "<+12>-12"],
]
mono_table(
    ["Zone", "Where it's used", "With DST", "Without DST (standard all year)"],
    tz_rows,
    widths=[1.3, 2.5, 1.6, 1.3],
)
small = doc.add_paragraph()
small.add_run(
    "Zones marked “(not used)” under With DST do not observe daylight saving time, "
    "so only the standard-time string applies. Daylight-saving rules shown reflect "
    "current U.S. law (spring forward 2nd Sunday of March, fall back 1st Sunday of "
    "November)."
).italic = True

doc.add_heading("Europe", level=2)
doc.add_paragraph(
    "Most of Europe observes EU daylight saving (forward on the last Sunday of "
    "March, back on the last Sunday of October). Countries that do not change "
    "their clocks are listed with a single standard-time value."
)
eu_rows = [
    ["Western European", "Portugal (mainland), Canary Islands",
     "WET0WEST,M3.5.0/1,M10.5.0", "WET0"],
    ["UK / Ireland", "United Kingdom, Ireland",
     "GMT0BST,M3.5.0/1,M10.5.0", "GMT0"],
    ["Central European",
     "France, Germany, Spain, Italy, Netherlands, Belgium, Poland, Sweden, Norway, "
     "Denmark, Switzerland, Austria, Czechia — most of central Europe",
     "CET-1CEST,M3.5.0,M10.5.0/3", "CET-1"],
    ["Eastern European",
     "Greece, Finland, Romania, Bulgaria, Estonia, Latvia, Lithuania, Ukraine",
     "EET-2EEST,M3.5.0/3,M10.5.0/4", "EET-2"],
    ["Azores", "Azores (Portugal)",
     "<-01>1<+00>,M3.5.0/0,M10.5.0/1", "<-01>1"],
    ["Iceland", "Iceland — no daylight saving",
     "(not used)", "GMT0"],
    ["Kaliningrad", "Kaliningrad, Russia — no daylight saving",
     "(not used)", "EET-2"],
    ["Moscow", "Moscow / western Russia — no daylight saving",
     "(not used)", "MSK-3"],
    ["Belarus", "Minsk, Belarus — no daylight saving",
     "(not used)", "<+03>-3"],
    ["Turkey", "Türkiye — no daylight saving",
     "(not used)", "<+03>-3"],
]
mono_table(
    ["Zone", "Where it's used", "With DST", "Without DST (standard all year)"],
    eu_rows,
    widths=[1.2, 2.6, 1.6, 1.3],
)
small = doc.add_paragraph()
small.add_run(
    "Zones marked “(not used)” under With DST do not change their clocks. POSIX "
    "values are taken directly from the IANA time zone database."
).italic = True

# ============================================================================
# 9. RESET WI-FI
# ============================================================================
doc.add_heading("9. Resetting Wi-Fi", level=1)
doc.add_paragraph(
    "If you move the device to a new network, or want to hand it to someone else, "
    "you can erase the saved Wi-Fi details:"
)
bullets([
    "Press and hold Button 3 for 3 to 10 seconds.",
    "The device erases its saved network and reboots.",
    "After rebooting it broadcasts the “ropener-XXXXXX” setup hotspot again — "
    "follow Section 2 to connect it to a network.",
])
note = doc.add_paragraph()
note.add_run("Note: ").bold = True
note.add_run(
    "This only clears Wi-Fi credentials. Your schedule, timezone, and tuning "
    "settings are kept."
)

# ============================================================================
# 10. ADVANCED / TUNING
# ============================================================================
doc.add_heading("10. Adjusting How It Moves (Advanced)", level=1)
doc.add_paragraph(
    "These settings are optional. The defaults work for a typical installation; "
    "change them only if needed. All values are remembered across reboots."
)
mono_table(
    ["Setting", "What it controls", "Notes"],
    [
        ["Centimeters", "Total travel distance of the curtain, in cm.",
         "Set this to your curtain's open-to-closed travel. Default 30 cm."],
        ["Speed", "How fast the motor runs (steps/second).",
         "Higher is faster but louder. Effective max is 2000."],
        ["Acceleration", "How quickly the motor speeds up / slows down.",
         "Higher is snappier; too high can skip or be noisy."],
        ["Motor Direction", "Which way the motor winds the rope.",
         "Flip this if Open and Close are reversed for your install."],
        ["IRUN value", "Motor running current / torque (1–31).",
         "Raise for more pulling force; lower if the driver runs hot. Default 25."],
        ["SGTHRS value", "Sensitivity of the stall detection used for homing (0–255).",
         "See tuning note below."],
        ["TCOOLTHRS value", "Speed threshold above which stall detection is active.",
         "See tuning note below."],
    ],
    widths=[1.4, 2.6, 2.4],
)

doc.add_heading("Tuning stall-based homing (optional)", level=2)
note = doc.add_paragraph()
note.add_run("Note: ").bold = True
note.add_run(
    "automatic stall detection is not working in the current firmware, so the "
    "SGTHRS / TCOOLTHRS settings have no effect yet — home manually by pressing "
    "Stop at the closed position (see Section 5). The guidance below applies once "
    "automatic detection is restored."
)
doc.add_paragraph(
    "Homing is meant to detect when the motor meets resistance at the closed end. "
    "Two readings on the web page help you tune it:"
)
bullets([
    ("SG_RESULT Sensor: ", "a live load reading that drops as the motor works harder."),
    ("TSTEP Sensor: ", "a measure of motor speed (smaller means faster)."),
])
doc.add_paragraph(
    "To tune: run the cover under normal load while watching SG_RESULT, note the "
    "value it settles at, then set SGTHRS to roughly half of that. If homing stops "
    "too early (false stalls), lower SGTHRS; if it does not detect the stop, raise "
    "it. Leave these at their defaults unless homing is unreliable."
)

# ============================================================================
# 11. HOME ASSISTANT
# ============================================================================
doc.add_heading("11. Home Assistant (Optional)", level=1)
doc.add_paragraph(
    "If you use Home Assistant, the device is discovered automatically over the "
    "local network and appears as a cover plus all the controls and readings in "
    "this guide. You do not need Home Assistant to use any feature — the web page "
    "offers the same controls."
)
doc.add_paragraph(
    "When first added, Home Assistant may warn that communication is not encrypted. "
    "The device still works normally. To remove the warning, an encryption key can "
    "be enabled in the firmware configuration (a setup step for the installer)."
)

# ============================================================================
# 12. TROUBLESHOOTING
# ============================================================================
doc.add_heading("12. Troubleshooting", level=1)
mono_table(
    ["Symptom", "What to try"],
    [
        ["Can't find the web page",
         "Make sure your phone/PC is on the same Wi-Fi as the device. Try the device's "
         "IP address instead of the .local name. Confirm it joined Wi-Fi (the setup "
         "hotspot should be gone)."],
        ["No setup hotspot appears",
         "Wait a full minute after power-on. If it already joined a network, hold Button 3 "
         "for 3–10 seconds to reset Wi-Fi and try again."],
        ["Open/Close are reversed",
         "Toggle the “Motor Direction” switch."],
        ["Position is wrong / drifted",
         "Run homing (Section 5) to recalibrate."],
        ["Schedule fires at the wrong time",
         "Check the Timezone value (Section 7/8) and that the device has internet for time sync."],
        ["Schedule doesn't run",
         "Confirm “Schedule Enabled” is ON and that Open/Close times are set."],
        ["Sun schedule opens/closes at the wrong time",
         "Set Latitude and Longitude for your location and check the Timezone "
         "(Section 7/8). With Sun Schedule ON, the fixed Open/Close times are ignored."],
        ["Homing motor doesn't stop on its own",
         "Expected in this version — automatic stop (StallGuard) is disabled. Press Stop "
         "when the curtain reaches the closed position (Section 5)."],
        ["Motor driver gets hot",
         "Lower the IRUN value."],
    ],
    widths=[2.1, 4.3],
)

# ============================================================================
# 13. QUICK REFERENCE
# ============================================================================
doc.add_heading("13. Quick Reference", level=1)
mono_table(
    ["Control (web page)", "Purpose"],
    [
        ["Ropener (cover)", "Open, close, set position, or stop."],
        ["State", "Shows IDLE / OPENING / CLOSING / HOMING."],
        ["Start-Stop Homing", "Start or cancel calibration."],
        ["Open Time / Close Time", "Daily schedule times (fixed-time mode)."],
        ["Schedule Enabled", "Turn the daily schedule on/off."],
        ["Sun Schedule", "Switch between fixed-time and sunrise/sunset scheduling."],
        ["Latitude / Longitude", "Your location used to compute sunrise/sunset."],
        ["Sun Open Offset / Sun Close Offset", "Minutes to shift open/close relative to sunrise/sunset."],
        ["Timezone", "Your local timezone (POSIX string)."],
        ["Centimeters", "Curtain travel distance."],
        ["Speed / Acceleration", "Motion tuning."],
        ["Motor Direction", "Reverse open/close direction."],
        ["IRUN value", "Motor torque / current."],
        ["SGTHRS / TCOOLTHRS", "Stall-detection tuning for homing."],
        ["SG_RESULT / TSTEP", "Live readings used when tuning homing."],
    ],
    widths=[2.4, 4.0],
)

mono_table(
    ["Physical button", "Action"],
    [
        ["Button 1 – short press", "Close (or stop if moving)."],
        ["Button 1 – long press", "Start/cancel homing."],
        ["Button 2 – release", "Open (or stop if moving)."],
        ["Button 3 – long press", "Reset Wi-Fi and reboot to setup."],
    ],
    widths=[2.4, 4.0],
)

out = r"c:\Github\Ropener\docs\Ropener User Guide.docx"
doc.save(out)
print("Saved:", out)
