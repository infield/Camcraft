## #714 - grblHAL post: A indexing about X, and the AltMill 4x4 + Vortex

**Decided:** the grblHAL post indexes a rotary A about X between
operations, with the same rules `tormach.cps` follows for the microARC 4
(#785). At a section's first motion it writes `G53 G0 Z<safe retract>`,
then `G0 A..` on its own line. The section's path is then posted in the
indexed frame: the tip relative to the work offset, turned by -A about X,
so WCS Y0 Z0 must lie on the A centerline (a note says so at the first
index). Every section restates A, upright ones too (`G0 A0`), because a
tool change or an operator stop between sections may have moved the
rotary. The post refuses by name: a tilt an A about X cannot reach (any B
or C), an A past a stated travel, an A that changes inside an operation
(simultaneous 4-axis), an arc or a canned cycle on an indexed section, an
index on a machine that states no safe Z retract, and a tilted move on a
machine with no A about X.

A new machine file, `sienci-altmill-4x4-vortex.mch` ("Sienci AltMill 4x4 +
Vortex"), is the 4x4 file plus an A rotary along X: `axis [1,0,0]`, no
travel stated (in gSender's 4th Axis Mode the Vortex turns without end),
A rate 8000 deg/min, `nose_to_axis` 55 mm and `axis_to_table` 97 mm. The
three 3-axis AltMill files are unchanged.

**Why:** a closed-loop Vortex on an AltMill's SLB-EXT is an independent A
output (Sienci's Vortex Quick Start Guide: "For Closed Loop Vortex, use 4th
Axis Mode"), along X (Sienci's software basics: orientation "Along X
Axis"), and Sienci's own setup zeroes Z on the axis center, which is the
indexed frame this post writes. Indexing needs no feed law for a combined
linear and rotary move, so it can be stated from public behavior alone;
wrapping cannot yet.

- **Lift in machine coordinates only.** Without a stated `safe_retract`
  this post lifts to a work-offset clearance height, and that height means
  nothing once the part has turned. All three AltMill files state
  `safe_retract` 0 (the AltMill homes), so the Vortex file inherits it.
- **Rate.** 8000 deg/min is the maximum rate Sienci recommends for the
  Vortex's A axis on other controllers ("Using other CNC routers"). An
  owner's `$113` replaces it.
- **`nose_to_axis` / `axis_to_table` are an owner's estimates** (axis 95
  to 100 mm over the wasteboard; nose 40 to 70 mm over the axis with Z
  homed), to be replaced by measured figures before this merges. Sienci's
  documents give no centre height. The only nearby figure is that the
  Vortex "requires 150mm (5.9") of height clearance from wasteboard to
  tool" for 4" stock, which is consistent with both estimates.
- **Direction sense** `[1,0,0]` is the RS-274 convention. It is still to
  be confirmed on a machine that a + A jog turns the tool right-handed
  about +X relative to the part.

**Considered and dropped:**
- Rotary wrapping (simultaneous A). grblHAL's feed for a move that
  combines linear and rotary axes, and whether it supports G93 inverse
  time, are not stated here. That stays refused by name, as on PathPilot.
- gSender's "Rotary Mode", which drives the Vortex from the Y output
  (the open-loop kit). There A replaces Y, so XYZ+A programs do not apply.
- Adding A to the plain 4x4 file. An owner without a Vortex would then get
  `G0 A0` in every program, and the swing gate would judge a rotary that
  is not there.

**Cost to change later:** the rotary figures are single numbers in one
file, pinned by `grblhal_fourth_axis.rs`. The golden
(`golden/nc/grblhal/index_4face_altmill_vortex.nc`) moves only if the
grblHAL post or the Vortex file changes. Wrapping would extend
`indexFor`'s refusal into a feed law of its own.
