# Ropener printed parts

One folder per hardware revision. **Print from the newest folder.** Older folders stay so
anyone who built an earlier revision can reprint a matching part.

```
plastics/
  v1.2.2/                      <- current
    body/        7 STEP        housing, arms, button, wall plate
    curtain/    11 STEP        carriages, clamps, end pulley, rope elbow
    platter-all-v1.2.2.3mf         generic 3MF: geometry + arrangement, opens in any slicer
    platter-all-v1.2.2-bambu.3mf   Bambu Studio project: same plate + P2S print settings (MakerWorld)
  v1.2.1/ v1.1.0/ v1.0.0/      <- previous revisions, frozen
```

The STEP files are the source of truth. Both 3MFs are **derived** from them and are
regenerated together whenever any STEP changes. Never edit a 3MF on its own.

## Print settings

| | |
|---|---|
| Material | PLA (any brand; white matches the kit) |
| Layer height | 0.24 mm (0.20 is fine, just slower) |
| Walls | 2 |
| Infill | 15 % |
| Supports | **None.** Every part is oriented to print without them. |
| Plate | one plate, all 18 parts, ~4.5 h on a Bambu P2S, ~8.75 h on a Prusa MK3 |
| Filament | ~150 g |

## Making a new revision

1. **Export STEPs from Fusion.** One file per body, same names as the previous folder, into
   `vX.Y.Z/body/` and `vX.Y.Z/curtain/`. Copy the unchanged parts from the previous folder
   rather than re-exporting them, so the diff shows only what really changed.
2. **Build the plate in Bambu Studio.** Import all 18 STEPs, arrange on one P2S plate,
   check every object's name is the part name (no `.step`, no old names).
   Process: 0.24 mm layer, 2 walls, 15 % infill, supports **off**. Filament: Generic PLA.
3. **Export the generic 3MF.** File → Export → Export Generic 3MF → `platter-all-vX.Y.Z.3mf`.
   This is the one for GitHub and Printables; PrusaSlicer, Orca and Cura all open it.
4. **Save the Bambu project.** File → Save Project As → `platter-all-vX.Y.Z-bambu.3mf`.
   This is the one MakerWorld needs for the print profile.
5. **Add a line to the changelog below.** What changed, which part, why.
6. **Commit.** `.gitattributes` stores STEP as LF and 3MF as binary; don't fight it.

Bump the patch number (1.2.1 → 1.2.2) when a part changes shape but nothing else in the
kit does. Bump the minor number when a hardware change also alters the BOM (new bearing,
new gear, different fasteners).

## After a new revision: the places that reference the version

- `README.md` (repo root) and `hardware/README.md` — current revision and the plate link
- MakerWorld print profile — replace the Bambu 3MF and the version in the profile name
- Printables — replace the generic 3MF and STEPs
- Wiki Build Guide — any step that shows the changed part
- Shopify — `The Ropener — Replacement 3D Printed Parts` prints from the current folder;
  product-page text still says "v1.1 hardware" and "9 hours (8h 44m)" as of this writing

## Changelog

### v1.2.2 — 2026-09-03
- `body/main-body.step` revised. **[fill in: what changed and why]**
- All other parts identical to v1.2.1.
- Prusa project 3MF dropped; one generic 3MF + one Bambu project 3MF from now on.
- `tube-cut-tool.step` is no longer on the plate (tubes ship pre-cut since v1.1).

### v1.2.1 — 2026-07-30
- **[fill in]** — first release with separate Bambu (P2S, 0.24 mm) and Prusa (MK3) plates.

### v1.1.0 — 2026-07
- MK7 → **MK8** drive gear (smaller diameter): pull 11 lb → **15 lb**.
- One **633ZZ** (3×13×5 mm) bearing per arm instead of two small bearings.
- PTFE tubes pre-cut (2×10, 2×40, 2×70 mm); plastics adjusted to suit; no razor cutting.
- 3M Command Strips replaced by 1" foam mounting squares (×5).

### v1.0.0 — 2026-06
- Initial public release of the redesigned Ropener.
