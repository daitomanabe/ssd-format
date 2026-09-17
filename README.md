# Spatial Scene Definition (SSD) / `.sscene`

`.sscene` is a plain-text, tab-delimited file format for describing spatial
installations: screens, surfaces, speakers, projectors, displays, cameras,
and the parent/child transform hierarchy between them. This repository
documents the format (profile `SSD_REVIEW_0.1`) and ships a small,
header-only C++17 reference reader plus example files.

This is a documentation/reference repository, not a full SDK. It records one
implementation's decisions where the original proposal left things open
(Euler convention, zero pose, etc.) — see "Open questions" below before
assuming interoperability with another SSD implementation.

## File format

- Plain UTF-8 text, LF or CRLF line endings, optional leading BOM.
- Fields are separated by literal **tabs**, not spaces.
- Lines starting with `#` are comments. Comment headers document column
  order for humans; they are not a column-remapping mechanism — columns are
  positional.
- The file is a sequence of `[SECTION]` headers followed by data rows.
- Section names and IDs are case-sensitive.
- Unknown sections and trailing extension columns are preserved on
  round-trip (lossless read/write of unrecognized data).

## Minimal example

```
[SCENE]
Version	0.1
Name	Example Room
Unit	meter
CoordinateSystem	SSD_RH_ZUP
AngleUnit	degree

[OBJECT]
# ID	Type	Name	Parent	X	Y	Z	Yaw	Pitch	Roll	Enabled
rig	custom	Rig	none	0	0	0	90	0	0	1
panel	screen	Panel	rig	0	3	1	0	90	0	1

[SCREEN]
# ID	Width	Height
panel	6	2
```

## Required `[SCENE]` header

| Key | Required value |
| --- | --- |
| `Version` | `0.1` |
| `Unit` | `meter` |
| `CoordinateSystem` | `SSD_RH_ZUP` |
| `AngleUnit` | `degree` |
| `Name` | optional, free text |
| `Profile` | optional; `SSD_REVIEW_0.1` declares this profile's conventions |

Other values are rejected, not silently converted. An empty scene (no
`[OBJECT]` rows) is valid.

## `[OBJECT]` — the core spatial hierarchy

```
[OBJECT]
# ID	Type	Name	Parent	X	Y	Z	Yaw	Pitch	Roll	Enabled
```

- `ID` is a string (small integers are conventional but not required).
- `Parent` is another OBJECT `ID`, or the literal `none`.
- Row order does not matter — a parent may appear after its children.
- Missing parents and reference cycles are errors. Maximum parent depth: 512.
- `Enabled=0` on an ancestor disables its descendants in a consuming viewer.

### Coordinate system and rotation

Right-handed, Z-up (`SSD_RH_ZUP`), meters, degrees. Column vectors, fixed
parent-axis rotation:

```
R = Ry(Roll) * Rx(Pitch) * Rz(Yaw)
M_local = T(X, Y, Z) * R
M_world = M_parent * M_local
```

Yaw (about +Z) is applied first to a local vector, then Pitch (about +X),
then Roll (about +Y). Translation is expressed in the parent's coordinate
frame and is not affected by the object's own rotation.

Rectangles (`SCREEN`/`SURFACE`/`LED`) have local **X = right, Y = up,
+Z = front normal**, centered on the object origin. At zero rotation a
rectangle lies flat in the world XY plane; `(Yaw, Pitch, Roll) = (0, 90, 0)`
stands it upright, facing world -Y.

```
local(U, V) = ((U - 0.5) * Width, (V - 0.5) * Height, 0)
world(U, V) = M_world * local(U, V)
```

`UV = (0, 0)` is the bottom-left corner, `(1, 1)` the top-right, in the
rectangle's own local frame.

Porting to another engine: convert the rotation matrix along with position —
`B * M * inverse(B)` for basis `B` — rather than swapping axis order on the
position alone and reusing the SSD Euler angles as-is.

## Device / output sections

Each row's leading `ID` references an `OBJECT` of the matching lowercase
`Type`, unless noted otherwise.

| Section | Columns | Notes |
| --- | --- | --- |
| `SCREEN` / `SURFACE` | `ID, Width, Height` | meters |
| `LED` | `ID, Width, Height, PixelWidth, PixelHeight, [PixelPitch]` | `type=led`; `PixelPitch` units are not yet standardized |
| `DISPLAY` | `ID, Name, Width, Height` | own ID namespace; positive integer pixels; no pose of its own |
| `PROJECTOR` | `ID, OutputID, TargetID, NativeWidth, NativeHeight` | `type=projector`; `OutputID` → `DISPLAY`, `TargetID` → a screen/surface/led object |
| `PIXELMAP` | `ID, DisplayID, TargetID, X, Y, Width, Height, Rotation` | own ID namespace; integer pixels; `Rotation` ∈ {0, 90, 180, 270} |
| `SPEAKER` | `ID, Channel, Gain, Delay, Mute` | `type=speaker`; positive integer channel, dB, non-negative ms, 0/1 |
| `CAMERA` | `ID, FovH, FovV, ResolutionX, ResolutionY` | `type=camera`; FOV in (0, 180) degrees |

`MICROPHONE`, `SENSOR`, `TRACKER`, `LIGHT`, and `ROBOT` exist in the
original proposal but have looser field definitions here — check
`include/ssd/Scene.h`'s validation branch before relying on them.

## Numeric formatting

Locale-independent decimal or scientific notation (`1.5`, `-2.5e-1`), period
decimal separator, no leading `+`, no hex, no `NaN`/`Infinity`. Integer
device/raster fields are limited to `0`–`2147483647` (or `1`–`2147483647`
where zero is not meaningful, e.g. pixel dimensions).

## Open questions (before treating two SSD implementations as interoperable)

The original proposal leaves several things unspecified; this profile
(`SSD_REVIEW_0.1`) makes a concrete choice for each, documented so it can be
renegotiated under a new `Profile` identifier rather than silently changed:

- Euler application order / intrinsic vs. extrinsic rotation
- The rectangle's zero-rotation pose (here: flat in world XY)
- Pixel-map origin and rotation pivot (`PIXELMAP` → `UV` is not defined)
- `PixelPitch` units for `LED`
- Optional-field schema for `MICROPHONE`/`SENSOR`/`TRACKER`/`LIGHT`/`ROBOT`

Never change one of these conventions while keeping the same `Profile`
identifier — mint a new one and version the change.

## Reference reader

`include/ssd/Scene.h` is a header-only C++17 parser: `ssd::load(path)` /
`ssd::parse(text)`, `scene.world(id)`, `scene.active(id)`,
`scene.uvToWorld(id, u, v)`. `serialize()` reproduces the original source
byte-for-byte (including BOM, line endings, comments, and unknown sections)
— it is not a general-purpose scene writer.

```bash
c++ -std=c++17 -I include your_consumer.cpp -o your_consumer
```

## Examples

- [`examples/parented-screen.sscene`](examples/parented-screen.sscene) —
  minimal parent/child transform fixture
- [`examples/screen-pose-contract.sscene`](examples/screen-pose-contract.sscene) —
  zero-pose and upright reference poses
- [`examples/routing-demo.sscene`](examples/routing-demo.sscene) — synthetic
  `PROJECTOR`/`DISPLAY`/`PIXELMAP`/`LED`/`SPEAKER` routing chain

## License

MIT — see [LICENSE](LICENSE).
