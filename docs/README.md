# Docs

This folder holds the **sources** for the Ropener documentation and the scripts
that render them. The rendered deliverables (`.docx` / `.pdf` / `.png`) are
**generated artifacts** — they are written to `dist/` and are **not** tracked in
git (see the root `.gitignore`). The print-ready finals are published to the
Valar Systems Google Drive, the same way firmware binaries go to GitHub Releases.

## What's tracked (source)

| File | Role |
| --- | --- |
| `BOM-V2.0.md` | Bill of Materials, human-readable source (also linked from the wiki) |
| `Ropener-User-Guide.md` | Markdown copy of the user guide |
| `make_bom.py` | Renders the BOM `.docx` |
| `make_box_insert.py` | Renders the box-insert `.png` + `.pdf` |
| `make_user_guide.py` | Renders the user-guide `.docx` |
| `valar-logo-black.png`, `valar-logo-white.png` | Brand assets used by the generators |
| `wiki-*.md` | Mirror of the GitHub wiki pages |

## What's generated (in `dist/`, gitignored)

- `Ropener Bill of Materials.docx`
- `Ropener Box Insert.png` / `Ropener Box Insert.pdf`
- `Ropener User Guide.docx`
- `Ropener Bill of Materials.pdf`, `Ropener User Guide.pdf` (see Word step below)

## Regenerating

```sh
pip install python-docx pillow segno      # one-time
py -3.13 make_bom.py
py -3.13 make_box_insert.py
py -3.13 make_user_guide.py
```

Each script writes into `dist/`.

- **Box insert** is fully reproducible on any platform — the script emits both
  the `.png` and the `.pdf` directly.
- **BOM** and **User Guide** scripts emit only `.docx`. To produce the matching
  `.pdf`, open the `.docx` in **Microsoft Word** and *Save As → PDF* (requires
  Windows + Word; this step is not reproducible in CI).

Then upload the finals from `dist/` to the Valar Drive.

## Known caveat

The user-guide content currently lives in **two** places: inline in
`make_user_guide.py` and again in `Ropener-User-Guide.md`; `make_bom.py`
likewise duplicates `BOM-V2.0.md` by hand. Keep them in sync when editing. The
proper long-term fix is to have the generators read the markdown directly so
there is a single source of truth.
