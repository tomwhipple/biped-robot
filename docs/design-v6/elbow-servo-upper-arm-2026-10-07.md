# The elbow servo moves into the upper arm (2026-10-07)

> Dated record: accurate as of its date; the current design is
> [DESIGN.md](../../DESIGN.md).

Tom: *"why are the bodies of the elbow servos in the forearms? I'd think the
upper arm would be a better place."* And, once the change was under way:
*"while you're working on the upper arm, also take a look at printability and
consider the same approach we settled on for the leg links."*

## Why the case was in the forearm

Nothing chose it. No record weighs the two placements. The get-up study's
round-2 plant hung a placeholder servo box on the forearm body, and the arm
CAD (`arms.md`, 2026-09-19) drew the study's result "as given", reusing the
leg's pattern: the distal link grips the case, the proximal link forks onto the
discs. In the leg that pattern is forced: one part, `leg_link_v6`, serves four
joints, so each copy grips one servo and forks onto the next. The upper arm
grips nothing at its shoulder end (the shoulder servo lives in the girdle), so
nothing forces the elbow servo onto the forearm.

## What changed

- **`arm_upper_v6`** grips the elbow servo's case: the leg link's grip
  channel, turned end for end about the elbow axis, so the case runs up the
  arm (output end 10.11 mm below the axis, cable end 35.11 mm above). The horn
  still faces outboard. The idler grip plate stops square 5 mm above its
  screws, as on the leg link.
- **`arm_fore_v6`** forks round the servo's horn and idler discs: the fork the
  upper arm used to carry, turned end for end. The arm plane, the disc screws
  and their stacks are unchanged.
- **Print, the leg-link approach** (9f9a146): both links are an open-front U
  printed web-down, with an **end wall** across each end of the open span
  against twist (plates in the X-Y plane, which print as walls rising off the
  bed). Upper arm: below the shoulder head's access bore, and 2 mm past the
  cable window above the elbow servo. Forearm: where the fork tines end; the
  hand knuckle closes the other end. The upper arm's head loses its **front
  wall** (Tom's 2026-09-24 request), which printed as a 6.4 × 48 mm bridge over
  a 23.5 mm cavity. The outboard flange keeps its front strip, so it is still a
  closed ring round the screw-access bore.
- The plant puts the elbow servo's 55 g on the upper-arm body
  (`DesignParams.arm_elbow_servo_upper`; get-up configs `r6_asdrawn[_rom120]`).
  The committed plant `sim/bimo_biped_v6ar.xml` is regenerated.

## Measured

| | before | after | how |
|---|---|---|---|
| `arm_upper_v6` / `arm_fore_v6` | 33.1 / 33.7 g | 27.8 / 41.5 g | `arm_v6.py` |
| printed, per arm | 66.7 g | 69.4 g | |
| robot (plant) | 2.269 kg | 2.274 kg | `build_v6_inertia.py` |
| arm inertia about the shoulder, hanging | 4.32 × 10⁻³ kg m² | 4.02 × 10⁻³ kg m² (−7 %) | the plant's `L_arm` + `L_forearm` |
| printability, upper arm | PASS; CEILING 309 mm²; slicer supports (elbow pads) | PASS; no ceiling; **no supports** | `check_printability` |
| printability, forearm | PASS; no supports | PASS; slicer supports (elbow pads, build plate only) | |
| head section modulus about X, through the access bore | 216 mm³ | 194 mm³ (−10 %) | slice of the solid at the shoulder axis |
| head, below the bore | 584 mm³ | 571 mm³; 667 at the end wall | slices at z −20 / −12 |
| elbow range, CAD | −101° … +60° | −118° … +60° | `arm_v6.check_elbow_rom` (declared range stays −100° … +10°) |
| shoulder → elbow lead: span + loop | 157.5 + 31.4 = 189 mm | 129.5 + 22.8 = 152 mm | `lead_lengths.py --assembly` |
| arm vs leg, rest pose, every leg joint's full ROM | ≥ 8.55 mm | ≥ 8.85 mm | `check_assembly_v6` (the CAD gate) |
| arm vs leg, rest pose, walking grid | ≥ 15.55 mm | ≥ 17.85 mm | [arm_walk_grid_2026-10-07.txt](arm_walk_grid_2026-10-07.txt) |
| hanging arm vs the swinging thigh | 20.45 mm (the upper arm's fork) | 20.51 mm (the forearm's fork) | `check_assembly_v6` |
| scripted get-up | 2/12 variants stand, both 6/6 robust | the same | [getup_search_r6_asdrawn_rom120.txt](getup_search_r6_asdrawn_rom120.txt) |
| get-up gate per servo set | 3250 / p3rk / p4rk 2/12, 6/6; stock 0/12 | the same, prone roll the same | [no3250_getup_elbow_upper.txt](no3250_getup_elbow_upper.txt) |
| walk at the rest pose | 0 arm-vs-leg contacts | the same verdicts, 0 contacts | [no3250_arms_walk_rest_folded_elbow_upper.txt](no3250_arms_walk_rest_folded_elbow_upper.txt) |

Every "before" is `main` 88f1e68 run the same day, so each row changes one
thing.

- **The lead.** It no longer crosses the elbow fold, and the span is 28 mm
  shorter: the connector trench sits 14 mm from the axis toward the cable end,
  which is now above the elbow, not below it. The table's spans are straight
  lines, so they never showed the old route, which went *down* the forearm to
  a window 60–72 mm below the elbow before coming back up across it. At 152 mm
  it is still the one hop over the stock 150 mm lead: plan on ~200 mm.
- **Inertia.** The servo alone would have taken ~10 % off; the forearm's print
  gained 7.8 g (the fork and its jog weigh more than the grip channel did), so
  the net is 7 %.
- **The get-up and the walk** are unchanged: the elbow servo moves 25 mm along
  an arm that is folded or pushing, and nothing in either gate is limited by it.

## What it costs, and what is open

- **The head is 10 % weaker through the access bore** without the front wall:
  194 mm³ there (was 216). A 20 N sideways knock at the hand puts ~33 MPa in
  that section (was ~30) and ~11 MPa in the section below it, in a ~50 MPa
  material. Tom asked for that front wall on 2026-09-24; the leg-link approach
  reverses it. Putting it back is one line in `arm_upper_v6`.
- **The elbow servo's encoder now reads the other way round** against the
  elbow angle: it measures horn against case, and horn and case have swapped
  links. Nothing committed encodes the arm servos' direction; `cal dir` is
  re-derived on the robot for every joint (docs/servo-map.md).
- **`ARM_REST` was not re-searched.** Shoulder −15° / elbow −95° was chosen on
  the old geometry; it clears by more now, so a smaller lean may also clear.
- **Assembly:** the elbow servo goes into the upper arm's channel from the
  front (step 18), then the forearm lifts onto the discs (step 19). The two
  inboard grip screws are driven from inboard, as the old fork's idler screws
  were.
