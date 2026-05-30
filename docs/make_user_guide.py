"""Generate the Ropener Curtain Controller user guide as a .docx file.

Run:  python make_user_guide.py
Output: "Ropener User Guide.docx" in this folder.
"""

from docx import Document
from docx.shared import Pt, RGBColor, Inches
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.enum.table import WD_TABLE_ALIGNMENT

ACCENT = RGBColor(0x1F, 0x4E, 0x79)   # dark blue
MONO = "Consolas"

doc = Document()

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
    t.style = "Light Grid Accent 1"
    t.alignment = WD_TABLE_ALIGNMENT.LEFT
    hdr = t.rows[0].cells
    for i, h in enumerate(headers):
        hdr[i].text = ""
        run = hdr[i].paragraphs[0].add_run(h)
        run.bold = True
    for row in rows:
        cells = t.add_row().cells
        for i, val in enumerate(row):
            cells[i].text = ""
            run = cells[i].paragraphs[0].add_run(val)
            # monospace any cell that looks like a POSIX string / value
            if any(ch in val for ch in (",", "<")) or val[:3] in (
                "EST", "CST", "MST", "PST", "AKS", "HST", "SST", "ChS", "AST", "UTC",
            ):
                run.font.name = MONO
                run.font.size = Pt(9.5)
    if widths:
        for row in t.rows:
            for i, w in enumerate(widths):
                row.cells[i].width = Inches(w)
    return t


# ============================================================================
# TITLE
# ============================================================================
title = doc.add_paragraph()
title.alignment = WD_ALIGN_PARAGRAPH.CENTER
r = title.add_run("Ropener Curtain Controller")
r.bold = True
r.font.size = Pt(26)
r.font.color.rgb = ACCENT

sub = doc.add_paragraph()
sub.alignment = WD_ALIGN_PARAGRAPH.CENTER
r = sub.add_run("User Guide")
r.font.size = Pt(15)
r.font.color.rgb = RGBColor(0x55, 0x55, 0x55)

ver = doc.add_paragraph()
ver.alignment = WD_ALIGN_PARAGRAPH.CENTER
ver.add_run("Firmware 2.3.0").italic = True

doc.add_paragraph()

intro = doc.add_paragraph()
intro.add_run(
    "Ropener is a Wi-Fi connected curtain/cover controller. It pulls a beaded "
    "rope with a small stepper motor to open and close your curtains. You can "
    "control it from any web browser on your network — no Home Assistant or "
    "cloud account required — or pair it with Home Assistant if you use one. "
    "It supports a daily open/close schedule, automatic position calibration, "
    "and several tuning options."
)

# ============================================================================
# 1. AT A GLANCE
# ============================================================================
doc.add_heading("1. At a Glance", level=1)
bullets([
    ("Control from a browser: ", "open the device's web page on your phone or computer."),
    ("Three ways to operate: ", "the web page, the physical buttons on the device, or Home Assistant."),
    ("Open / close / partial: ", "drive the curtain fully open, fully closed, or to any position in between."),
    ("Daily schedule: ", "set a time to open and a time to close automatically every day."),
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
    "Connect to it. A setup page should open automatically (this is the “captive "
    "portal”). If it does not, open a browser and go to http://192.168.4.1.",
    "Choose your home Wi-Fi network from the list, enter its password, and save.",
    "The device will reboot and join your network. The setup hotspot disappears once "
    "it connects successfully.",
])

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
doc.add_paragraph(
    "“Homing” teaches the device exactly where the fully-closed position is. The "
    "motor gently drives the curtain toward the closed end until it feels the rope "
    "reach its stop, then marks that spot as 0% (closed). Everything else — open "
    "position, percentages, and the schedule — is measured from there."
)
doc.add_paragraph("Run homing when:")
bullets([
    "You first install the device.",
    "The reported position no longer matches reality (for example after the rope "
    "slipped or was pulled by hand).",
])
doc.add_heading("How to start homing", level=2)
bullets([
    "Press and hold Button 1 for 3 to 10 seconds, or",
    "Click the “Start-Stop Homing” button on the web page.",
])
doc.add_heading("How to stop or cancel homing", level=2)
bullets([
    "Press Button 1 again, or",
    "Click “Start-Stop Homing” again.",
])
doc.add_paragraph(
    "The “State” reading on the web page shows HOMING while calibration is in "
    "progress and returns to IDLE when finished."
)

# ============================================================================
# 6. DAILY SCHEDULE
# ============================================================================
doc.add_heading("6. Set Up the Daily Schedule", level=1)
doc.add_paragraph(
    "The device can open and close the curtain automatically at set times every "
    "day. Four controls on the web page work together:"
)
mono_table(
    ["Control", "What to set"],
    [
        ["Open Time", "The time of day to open the cover (e.g. 07:00)."],
        ["Close Time", "The time of day to close the cover (e.g. 21:00)."],
        ["Schedule Enabled", "Turn this ON to run the schedule, OFF to pause it."],
        ["Timezone", "Your local timezone, so the times mean what you expect (see Section 7)."],
    ],
    widths=[1.7, 4.7],
)
doc.add_paragraph("Steps:")
bullets([
    "Set your Timezone first (Section 7) — the open/close times are in local time.",
    "Set Open Time and Close Time.",
    "Turn Schedule Enabled ON.",
], style="List Number")
note = doc.add_paragraph()
note.add_run("Notes: ").bold = True
note.add_run(
    "The schedule needs an internet connection to keep accurate time. Scheduled "
    "moves are skipped while the device is homing. The schedule is off by default, "
    "and all four settings are remembered across reboots."
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
    "The two trailing parts (for example “,M3.2.0,M11.1.0”) are the daylight-saving "
    "rules: switch forward on the 2nd Sunday of March and back on the 1st Sunday of "
    "November — the current U.S. rule. If your area does not observe daylight saving "
    "time, use the “without DST” value, which has no trailing rules."
)

# ============================================================================
# 8. US TIMEZONE TABLE
# ============================================================================
doc.add_heading("8. U.S. Timezone POSIX Strings", level=1)
doc.add_paragraph(
    "Find your region below and copy the matching string into the Timezone box. "
    "Use the “With DST” column if your area changes its clocks twice a year; use "
    "“Without DST (standard all year)” if it does not."
)

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
doc.add_paragraph(
    "Homing works by detecting when the motor meets resistance at the closed end. "
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
        ["Homing stops too early or not at all",
         "Adjust SGTHRS (Section 10, tuning note)."],
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
        ["Open Time / Close Time", "Daily schedule times."],
        ["Schedule Enabled", "Turn the daily schedule on/off."],
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
