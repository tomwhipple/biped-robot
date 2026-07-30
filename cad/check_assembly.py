"""Assembly interference checks: mock servo bodies + printed parts, boolean
intersection volumes at joint-range extremes. All volumes should be ~0 mm^3.

Run:  .venv/bin/python cad/check_assembly.py
"""
from build123d import *
import dimensions as D
import parts
import fasteners as F


def servo_mock_y():
    """Servo with output axis along +Y at origin (leg pitch joints).
    Body length along Z: top end +10.11, bottom -35.11; horn cylinder and the
    O25 x 0.55 idler recess (which our idler bosses reach into) included.

    Carries the two vendor-STEP features the earlier block-and-recess mock left
    out (2026-07-28): the HORN-SIDE RIB on the cable half, and the rotating
    IDLER HUB inside the recess. Without them this mock reported clearance where
    the real servo interferes -- leg_link overlaps the rib by ~299 mm3."""
    # CASE STOPS AT SV_IDLER_CASE_FACE, NOT SV_BOTFACE (2026-07-29). The case is
    # NOT symmetric about the output axis: slab-sectioning the vendor solid finds
    # zero material at -17.35, and the real idler-side face is 2.60 mm in, at
    # -14.75. The old block ran the case all the way to -17.35, which invented a
    # seating surface that does not exist -- every mount that "seats" on the
    # idler side was really being checked against phantom material.
    case = parts.box(-12.36, 12.36, D.SV_IDLER_CASE_FACE, D.SV_TOPFACE,
                     -D.SV_AXIS_FROM_REAR, D.SV_AXIS_FROM_OUT_END)
    # horn-side rib: cable end is -Z in this frame
    case += parts.box(-D.SV_HORN_RIB_HW, D.SV_HORN_RIB_HW,
                      D.SV_TOPFACE, D.SV_TOPFACE + D.SV_HORN_RIB_H,
                      -D.SV_HORN_RIB_L[1], -D.SV_HORN_RIB_L[0])
    # moulded back-cover platform, 1.90 proud of that face
    case += parts.box(-D.SV_IDLER_BOSS_HW, D.SV_IDLER_BOSS_HW,
                      D.SV_IDLER_BOSS_Y, D.SV_IDLER_CASE_FACE,
                      D.SV_IDLER_BOSS_Z[0], D.SV_IDLER_BOSS_Z[1])
    # idler disc stands PROUD of the case face (it is not recessed into it), and
    # the free hub proud of the disc again. Both ROTATE with the joint.
    case += parts.cyl_y(D.SV_HORN_D / 2, D.SV_IDLER_FACE, D.SV_IDLER_CASE_FACE, 0, 0)
    case += parts.cyl_y(D.SV_IDLER_HUB_HW, D.SV_BOTFACE, D.SV_IDLER_FACE, 0, 0)
    horn = parts.cyl_y(D.SV_BOSS_D / 2, D.SV_TOPFACE, D.SV_HORN_FACE, 0, 0)
    return case + horn


def servo_mock_x():
    """Roll servo: output axis +X, body length along Z (output end DOWN):
    axis at z=0, top end +35.11, bottom -10.11. Idler face (-X) carries the
    measured SV_CONN trench and the free-hub post at its STEP-worst-case
    protrusion (must stay inside the bay bore)."""
    case = parts.box(D.SV_BOTFACE, D.SV_TOPFACE, -12.36, 12.36,
                     -D.SV_AXIS_FROM_OUT_END, D.SV_AXIS_FROM_REAR)
    case -= parts.cyl_x(D.SV_IDLER_RECESS_D / 2, D.SV_BOTFACE - 0.01,
                        D.SV_IDLER_FACE, 0, 0)
    case -= parts.box(D.SV_BOTFACE - 0.01, D.SV_BOTFACE + D.SV_CONN_FLOOR,
                      -D.SV_CONN_HW, D.SV_CONN_HW,
                      D.SV_CONN_L[0], D.SV_CONN_L[1])
    case += parts.cyl_x(D.SV_IDLER_HUB_HW, D.SV_BOTFACE, D.SV_IDLER_FACE, 0, 0)
    # horn-side rib: in this frame the cable end is +Z
    case += parts.box(D.SV_TOPFACE, D.SV_TOPFACE + D.SV_HORN_RIB_H,
                      -D.SV_HORN_RIB_HW, D.SV_HORN_RIB_HW,
                      D.SV_HORN_RIB_L[0], D.SV_HORN_RIB_L[1])
    horn = parts.cyl_x(D.SV_BOSS_D / 2, D.SV_TOPFACE, D.SV_HORN_FACE, 0, 0)
    return case + horn


def servo_mock_z():
    """Yaw servo (v3yaw): output axis VERTICAL, horn DOWN. Case length along X
    (YAW_CASE_X_REAR..FRONT), width along Y, thickness along Z. Origin at the
    output axis on the case mid-plane; idler-side face UP (recessed O25 disc
    well, measured SV_CONN trench, stator-boss recesses, and the free-hub post
    at its STEP-worst-case protrusion -- the deck pocket must clear it), horn
    boss DOWN."""
    case = parts.box(D.YAW_CASE_X_REAR, D.YAW_CASE_X_FRONT, -12.36, 12.36,
                     D.SV_BOTFACE, D.SV_TOPFACE)
    case -= parts.cyl_z(D.SV_IDLER_RECESS_D / 2, 16.80, D.SV_TOPFACE + 0.01, 0, 0)
    case -= parts.box(-D.SV_CONN_L[1], -D.SV_CONN_L[0],
                      -D.SV_CONN_HW, D.SV_CONN_HW,
                      D.SV_TOPFACE - D.SV_CONN_FLOOR, D.SV_TOPFACE + 0.01)
    for xrow in D.YAW_CASE_HOLES_IDLER:            # stator screw boss recesses
        for s in (1, -1):
            case -= parts.cyl_z(3.7, D.SV_TOPFACE - D.SV_IDLER_BOSS_RECESS,
                                D.SV_TOPFACE + 0.01, -xrow, s * D.CASE_HOLE_LAT)
    case += parts.cyl_z(D.SV_IDLER_HUB_HW, 16.80, D.SV_TOPFACE, 0, 0)
    # horn-side rib: for the yaw mock the horn side is -Z and the cable end -X
    case += parts.box(-D.SV_HORN_RIB_L[1], -D.SV_HORN_RIB_L[0],
                      -D.SV_HORN_RIB_HW, D.SV_HORN_RIB_HW,
                      D.SV_BOTFACE - D.SV_HORN_RIB_H, D.SV_BOTFACE)
    horn = parts.cyl_z(D.SV_BOSS_D / 2, -D.SV_HORN_FACE, D.SV_BOTFACE, 0, 0)
    return case + horn


def vol(a, b):
    i = a & b
    return 0.0 if i is None else i.volume


def check(label, v, tol=1.0):
    print(f"  {label:58s} {v:8.2f} mm3  {'OK' if v <= tol else '** COLLISION **'}")
    return v <= tol


def require(label, v, floor_mm3):
    """Inverse of check(): material that MUST be present. A pocket that has
    quietly lost its floor, or a screw boss with nothing to tap into, is just
    as broken as a collision and check() would call it OK."""
    print(f"  {label:58s} {v:8.2f} mm3  {'OK' if v >= floor_mm3 else '** MISSING **'}")
    return v >= floor_mm3


def clearance(label, a, b, need=D.SWEEP_BUFFER):
    """Minimum distance between two bodies that MOVE relative to each other:
    volume==0 only proves they don't overlap, not that they'd survive print
    tolerance + joint play. ROM extremes get SWEEP_BUFFER of real air (user
    call 2026-07-28). Designed FITS (boss-in-bore, seat collars) are exempt
    -- a slip fit is supposed to be closer than the buffer."""
    d = a.distance_to(b)
    print(f"  {label:58s} {d:8.2f} mm   "
          f"{'OK' if d >= need else '** UNDER BUFFER **'}")
    return d >= need



# ---------------------------------------------------------------- insertion
# Sampling step for the COMPONENT sweeps below. A true swept solid (Minkowski
# sum along the axis) is neither cheap nor robust in OCC for a mock this
# lumpy, so the servo is instead PLACED at interpolated offsets and intersected
# at each one -- stepped placement sampling. 0.5 mm is fine enough that a 1 mm
# ledge is hit at least twice, and the whole sweep costs ~2 s per component
# (~0.02 s a boolean).
INSERT_STEP = 0.5

# Real insertion direction for every servo that has to be got INTO a printed
# part, as (part-local unit axis, travel needed to be fully clear). Direction
# is the servo's motion AWAY from its seat, i.e. the approach runs backwards
# along it. Sources are the parts.py comments that settled each cavity:
#
#   leg_link  the grip channel is a C-section closed by the web on -x and by
#             the two grip plates on +/-y, so the ONLY free axis is z: the case
#             enters through the channel's top mouth (GRIP_TOP_HORN, just under
#             the horn) and slides down. Equivalently, and how it is actually
#             done on the bench: slide the link up onto the case starting at
#             the case's free (cable) end. Sliding it on "from the front", as
#             docs/assembly.md used to say, drives the horn disc straight into
#             the grip plate -- 185 mm3 at 8 mm in (2026-07-30).
#   foot      "the servo drops in vertically" (parts.foot, idler platform
#             window) -- +z, straight down into the pocket.
#   carrier   "downward-open U-slot ... slide each hip-roll servo up into it"
#             (docs/assembly.md 7c, parts.yaw_carrier) -- so it withdraws -z.
#   pelvis    "press it up into its collar on the deck underside" (7a) -- -z.
SERVO_INSERT = {"leg_link": ((0, 0, 1), 32.0),
                "foot": ((0, 0, 1), 24.0),
                "yaw_carrier": ((0, 0, -1), 30.0),
                "pelvis": ((0, 0, -1), 24.0)}


def insert_scan(part, mock, axis, travel, step=INSERT_STEP):
    """Worst intersection of `mock` with `part` as the mock travels `travel`
    along `axis` from its seated pose. Returns (worst_mm3, offset_at_worst)."""
    worst, at = 0.0, 0.0
    n = int(round(travel / step))
    for i in range(1, n + 1):
        d = i * step
        v = vol(part, Pos(axis[0] * d, axis[1] * d, axis[2] * d) * mock)
        if v > worst:
            worst, at = v, d
    return worst, at


def check_path(label, part, mock, key, tol=1.0):
    w, at = insert_scan(part, mock, *SERVO_INSERT[key])
    print(f"  {label:58s} {w:8.2f} mm3  "
          f"{'OK' if w <= tol else '** BLOCKED **'} (worst at {at:+.1f} mm)")
    return w <= tol


def main():
    """Run every interference check. Behind a main() since 2026-07-27: this
    file's checks used to execute at IMPORT, so every script that imported it
    merely for the servo mocks (export_assembly, export_assembly_full, and
    thence every render script) silently ran the full ~17 s suite first."""
    ok = True
    print("== leg_link vs its own (gripped) servo ==")
    ll = parts.leg_link()
    ok &= check("neutral (contact only)", vol(ll, servo_mock_y()))

    print("== leg_link vs the NEXT servo across the lower joint (knee/ankle) ==")
    for ang in (0, 5, -40, -60, -95, 40, 60, 95):
        sv = Pos(0, 0, -D.LINK_DROP) * Rot(0, ang, 0) * servo_mock_y()
        ok &= check(f"lower servo at {ang:+d} deg", vol(ll, sv))

    print("== leg_link vs the yoke/link arms of the joint ABOVE it ==")
    # upper joint arms: use the REAL worst case -- a leg_link fork profile
    # (narrow slab to +40, wide-to-web slab +40..+57) plus pads and idler boss.
    web_x0 = -12.36 - D.WEB_GAP - 2.4
    arm_h, arm_i = None, None
    for a0, a1 in ((D.SV_HORN_FACE, D.SV_HORN_FACE + D.PLATE),
                   (D.IDLER_ARM_INNER - D.PLATE, D.IDLER_ARM_INNER)):
        a = parts.box(D.FORK_NARROW_X, 12, a0, a1, 0, -D.FORK_WIDE_Z + D.LINK_DROP) \
            + parts.box(web_x0, 12, a0, a1, -D.FORK_WIDE_Z + D.LINK_DROP, 57) \
            + parts.cyl_y(D.PAD_D / 2, a0, a1, 0, 0)
        arm_h = a if arm_h is None else arm_h + a
    arm_i = parts.cyl_y(D.IDLER_BOSS_D / 2, D.SV_IDLER_FACE, D.IDLER_ARM_INNER, 0, 0)
    for ang in (0, 60, -60, 95, -95):
        arms = Rot(0, ang, 0) * (arm_h + arm_i)
        ok &= check(f"upper arms at {ang:+d} deg", vol(ll, arms))

    print("== hip yokes + thigh servo vs yaw_carrier + roll servo, roll -25..+25 ==")
    # v3yaw: the roll joint moved from the pelvis bay onto yaw_carrier (identical
    # bay geometry). Roll clearance is now proven against the CARRIER, in its frame
    # (roll axis == X at CARRIER_ROLL_AXIS). The pelvis deck is ~40 mm higher and
    # trivially clear (checked once, below).
    pv = parts.pelvis()
    yc = parts.yaw_carrier()
    yr, yp = parts.yoke_roll(), parts.yoke_pitch()
    thigh_sv = servo_mock_y()
    roll_axis = D.CARRIER_ROLL_AXIS                     # carrier frame, axis along X
    roll_sv = Pos(0, 0, roll_axis) * servo_mock_x()
    ok &= check("roll servo vs carrier (contact only)", vol(yc, roll_sv))
    for ang in (0, 25, -25):
        # yoke_roll frame == roll axis at origin along X
        hip = yr + Pos(0, 0, -D.ROLL_TO_PITCH) * yp \
                 + Pos(0, 0, -D.ROLL_TO_PITCH) * thigh_sv
        hip = Pos(0, 0, roll_axis) * Rot(ang, 0, 0) * hip
        ok &= check(f"hip assy at roll {ang:+d} vs carrier", vol(yc, hip))
        ok &= check(f"hip assy at roll {ang:+d} vs roll servo", vol(hip, roll_sv))
    # buffer at the roll ROM extremes. The yoke_roll idler boss is EXCLUDED:
    # its 0.3 radial slip in the bay bore is the designed bearing fit, and a
    # 0.5 buffer would (correctly, uselessly) flag it. Everything else --
    # arms, flange, screws, the carried yoke_pitch -- gets the full buffer.
    yr_nb = yr - parts.cyl_x(10.5, -20.96, -16.7, 0, 0)
    hip_pr = yr_nb + F.disc_screws_x() + F.flange_bolts() \
        + Pos(0, 0, -D.ROLL_TO_PITCH) * (yp + F.disc_screws_y())
    carrier_scr = yc + F.yaw_carrier_screws()
    for ang in D.ROM["hip_roll"]:
        ok &= clearance(f"buffer: roll {ang:+.0f}: hip stack vs carrier+screws",
                        carrier_scr,
                        Pos(0, 0, roll_axis) * Rot(ang, 0, 0) * hip_pr)

    print("== HIP YAW (v3yaw): yaw servo + carrier + roll servo vs pelvis / other leg ==")
    YAW_MID = D.YAW_IDLER_FACE_Z - D.SV_TOPFACE          # -22.35, mock mid in pelvis frame


    def yaw_stack(by, yaw):
        """yaw_carrier + its hip-roll servo, yawed about the leg's vertical axis and
        placed under the deck at leg offset by (pelvis frame)."""
        c = yc + Pos(0, 0, D.CARRIER_ROLL_AXIS) * servo_mock_x()
        return Pos(0, by, D.YAW_HORN_FACE_Z) * Rot(0, 0, yaw) * c


    ysv = Pos(0, D.HIP_SEP / 2, YAW_MID) * servo_mock_z()
    ok &= check("yaw servo seated in pelvis (contact only)", vol(pv, ysv))
    ok &= check("roll servo vs carrier bay (contact only)",
                vol(yc, Pos(0, 0, D.CARRIER_ROLL_AXIS) * servo_mock_x()))
    for yaw in (0, 45, -45):
        stack = yaw_stack(D.HIP_SEP / 2, yaw)
        ok &= check(f"carrier+roll servo at yaw {yaw:+d} vs pelvis", vol(pv, stack))
        ok &= check(f"carrier+roll servo at yaw {yaw:+d} vs yaw servo", vol(stack, ysv))
    # both carriers yawed INWARD (worst mutual approach at the design +/-45 sweep)
    ok &= check("both carriers yawed inward 45 vs each other",
                vol(yaw_stack(D.HIP_SEP / 2, -45), yaw_stack(-D.HIP_SEP / 2, 45)))
    # buffer at the yaw ROM extremes, wall-screw heads aboard (the yaw servo
    # is excluded: the carrier plate CONTACTS its horn face by design)
    def yaw_stack_scr(by, yaw):
        c = yc + F.yaw_carrier_screws() \
            + Pos(0, 0, D.CARRIER_ROLL_AXIS) * servo_mock_x()
        return Pos(0, by, D.YAW_HORN_FACE_Z) * Rot(0, 0, yaw) * c
    pv_scr = pv + F.deck_stator_screws()
    for yaw in D.ROM["hip_yaw"]:
        ok &= clearance(f"buffer: yaw {yaw:+.0f}: carrier stack vs pelvis",
                        pv_scr, yaw_stack_scr(D.HIP_SEP / 2, yaw))
    ok &= clearance("buffer: both stacks yawed inward 45",
                    yaw_stack_scr(D.HIP_SEP / 2, -45),
                    yaw_stack_scr(-D.HIP_SEP / 2, 45))

    print("== FASTENERS seated in their own parts (recesses swallow the heads) ==")
    # Every screw is now a real solid (fasteners.py) placed in its host part's
    # frame. A screw fully swallowed by its clearance hole + countersink /
    # counterbore intersects its host at 0 mm3 -- so this one boolean per part
    # verifies hole diameters, countersink depths AND counterbore depths at
    # once. History: fasteners used to exist only as symbols in the docs, and
    # that blindness cost three bench finds (yaw-horn heads vs roll servo,
    # PAD_D vs grip heads, and the 2026-07-28 leg-link skew below).
    grip = F.leg_link_screws()
    ok &= check("leg_link vs its 6 flush grip screws", vol(ll, grip))
    ok &= check("pelvis vs 8 counterbored stator screws", vol(pv, F.deck_stator_screws()))
    ok &= check("yaw_carrier vs its 12 screws", vol(yc, F.yaw_carrier_screws()))
    ok &= check("yoke_pitch vs its 8 disc screws", vol(yp, F.disc_screws_y()))
    ok &= check("yoke_roll vs disc screws + flange bolts",
                vol(yr, F.disc_screws_x()) + vol(yr, F.flange_bolts()))
    ft = parts.foot()
    ok &= check("foot vs its 4 tab screws", vol(ft, F.foot_screws()))
    ok &= check("tower vs feet + board screws", vol(parts.tower(), F.tower_screws()))
    # deck-top counterbore must sink the pan head sub-flush: the battery pack
    # sits flat on the deck and its footprint covers the -8.30 stator row
    _batt = parts.box(D.BATT_SEAT_X, D.BATT_SEAT_X + D.BATT[1],
                      -D.BATT[0] / 2, D.BATT[0] / 2, 0, D.BATT[2])
    ok &= check("battery footprint vs stator screw heads",
                vol(_batt, F.deck_stator_screws()))
    _sink = D.DECK_CB_DEPTH - D.CASE_HEAD_H
    print(f"  {'stator head below deck top (cb depth - head height)':58s} "
          f"{_sink:8.2f} mm   {'OK' if _sink >= 0.2 else '** PROUD **'}")
    ok &= _sink >= 0.2

    print("== SCREW INSERTION PATHS (can the screw + driver REACH the seat?) ==")
    # The block above proves each head is SWALLOWED once seated. This one
    # proves it can get there: fasteners.py sweeps the head/driver envelope
    # back along every screw axis, INSERT_LEN out of the part, and the host
    # part must not stand in that air. Voids (bore, countersink, counterbore,
    # access holes) are voids, so a clean part reads ~0 mm3.
    #
    # This is the check that was missing on 2026-07-30, when the leg_link's two
    # idler grip countersinks turned out to be buried behind the jog block and
    # the fork wide plate (3.1 mm proud of the seat, cutting across the lower
    # half of both head circles). Remove a4ce69e's two access counterbores and
    # the first line below goes to ~78 mm3.
    #
    # Each sweep is checked against the part it is driven INTO/THROUGH at that
    # step, not against everything that will eventually be around it: assembly
    # ORDER is what keeps e.g. the yaw-horn bolts (§7b) clear of the roll servo
    # that arrives in §7c, and the IMU screws (§9b) clear of the gopro_base.
    tw = parts.tower()
    ic = parts.imu_carrier()
    for label, host, group in (
            ("leg_link: 6 grip screws", ll, "screws_grip"),
            ("foot: 4 tab screws", ft, "screws_foot"),
            ("pelvis: 8 stator screws down through the deck", pv, "screws_deck"),
            ("yaw_carrier: 8 wall screws", yc, "screws_yaw_wall"),
            ("yaw_carrier: 4 horn bolts (bay still empty, §7b)", yc,
             "screws_yaw_horn"),
            ("yoke_pitch: 8 disc screws", yp, "screws_disc_y"),
            ("leg_link fork: the same 8 disc screws one joint down",
             Pos(0, 0, D.LINK_DROP) * ll, "screws_disc_y"),
            ("yoke_roll: 8 disc screws", yr, "screws_disc_x"),
            ("yoke_roll: 4 flange bolts", yr, "screws_flange"),
            ("tower: 4 feet bolts + 4 board screws", tw, "screws_tower"),
            ("imu_carrier: gopro bolts + IMU screws (§9b, base off)", ic,
             "screws_head_stack")):
        ok &= check(f"path: {label}", vol(host, F.paths(group)))
    # ANTI-DRIFT: fasteners.py's seat tables restate the head coordinates of
    # the screw builders. Every seat probe must land in head metal -- if a
    # screw moves and its seat does not (or vice versa), this fails instead of
    # the sweep quietly checking empty air.
    for group, (seats, screws) in F.SEATS.items():
        n = len(seats())
        hit = sum(1 for s in seats()
                  if vol(screws(), Pos(s[0], s[1], s[2]) * Sphere(0.3)) > 0)
        ok &= require(f"{group}: sweep mouths landing on a head ({hit}/{n})",
                      float(hit), float(n))

    print("== COMPONENT INSERTION PATHS (can the SERVO reach its seat?) ==")
    # Same blindness, one size up: every "contact only" check above proves the
    # servo FITS where it ends up, never that it can be brought there. Stepped
    # placement sampling along the real insertion axis (see SERVO_INSERT).
    ok &= check_path("leg_link: gripped servo out through the channel mouth",
                     ll, servo_mock_y(), "leg_link")
    ok &= check_path("yaw_carrier: roll servo down out of the U-slot",
                     yc, Pos(0, 0, D.CARRIER_ROLL_AXIS) * servo_mock_x(),
                     "yaw_carrier")
    ok &= check_path("pelvis: yaw servo down off its deck collar",
                     pv, Pos(0, D.HIP_SEP / 2, YAW_MID) * servo_mock_z(),
                     "pelvis")
    # ...and the ankle servo, which does NOT come clean. 2026-07-30: the
    # servo's moulded horn-side RIB is 1.13 mm proud of SV_TOPFACE and the +Y
    # retention tab's inner face is only 0.30 off that same face, so 0.83 mm of
    # rib has to travel down the tab. Its detent (parts.foot) is a CLOSED
    # pocket in z -- it has to be: opening it out the top, the way the
    # idler-side platform window is open, would run the relief straight through
    # the z 26.61 screw's bearing land, and every millimetre of tab above the
    # rib band is swept during a vertical drop-in, so there is no shape that
    # both passes the rib and backs that screw. What the number really means is
    # a SNAP: a 0.83 x 8.3 mm section springing a 2.4 mm x 26 mm PETG
    # cantilever tab, on a foot that has been printed and assembled with a real
    # servo in it. And parts.foot's own NOTE says the two sources disagree
    # about whether that rib touches the tab at all (the placed vendor solid
    # reads 0.2 mm of CLEARANCE where the dimensions read 0.8 of interference).
    # So it is reported, not failed, until somebody puts calipers on the rib.
    _ankle_sv = Pos(0, 0, D.FOOT_T - D.FOOT_POCKET_D + 12.36) \
        * Rot(0, 90, 0) * servo_mock_y()        # as placed below, lying flat
    _rib_snap, _at = insert_scan(ft, _ankle_sv, *SERVO_INSERT["foot"])
    print(f"  {'foot: ankle servo drop-in (horn-rib snap past +Y tab)':58s} "
          f"{_rib_snap:8.2f} mm3  SNAP at {_at:+.1f} mm -- measure the rib")

    print("== GRIP-SCREW HEADS vs the arms that sweep them (the bench skew) ==")
    # 2026-07-28 bench find: the thigh links SKEWED on their servos. Cause: a
    # proud pan head (O5.0 x 2.0) on the horn grip plate sits in the 0.70 mm
    # band the yoke/fork arm above sweeps through -- overlap reached 25.5 mm3
    # from hip +/-60 deg on, so the arm rode the heads and pried the link
    # sideways. Fix: 90-deg countersinks + M2.5 FLAT-head self-tappers, heads
    # flush; these checks hold the whole ROM plus the SWEEP_BUFFER.
    lo, hi = D.ROM["hip_pitch"]
    for ang in (lo, -95, -60, 0, hi):
        ok &= check(f"grip screws at hip {ang:+.0f} vs yoke_pitch",
                    vol(yp, Rot(0, ang, 0) * grip))
    for ang in (lo, hi):
        ok &= clearance(f"buffer: grip screws at hip {ang:+.0f} vs yoke_pitch",
                        yp, Rot(0, ang, 0) * grip)
    # same story one joint down: the thigh's fork arms sweep the SHIN's grip
    # screws through the knee ROM (worst-case fork profile from the arm block)
    knee_arms = arm_h + arm_i + F.disc_screws_y()
    for ang in D.ROM["knee"]:
        ok &= check(f"grip screws at knee {ang:+.0f} vs fork arms+discs",
                    vol(Rot(0, ang, 0) * knee_arms, grip))
        ok &= clearance(f"buffer: knee {ang:+.0f} arms+discs vs shin+grip",
                        Rot(0, ang, 0) * knee_arms, ll + grip)

    print("== yaw-horn + flange SCREW HEADS vs the roll servo ==")
    # The 4x yaw-horn bolts pass UP through the carrier's mount plate into the
    # horn disc, so their HEADS sit proud of the bay ceiling -- pointing at the
    # roll servo's rear face (the original CARRIER_ROLL_HEAD_CLEAR find: the
    # servo used to sit flush against that ceiling and the heads buried 168 mm3
    # into it). The flange bolts point up at the servo's output end similarly.
    # heads only: the WALL screws' shanks intentionally thread into the case
    _roll_sv = Pos(0, 0, D.CARRIER_ROLL_AXIS) * servo_mock_x()
    ok &= check("yaw-horn screws vs roll servo", vol(F.yaw_horn_screws(), _roll_sv))
    _gap = D.CARRIER_ROLL_CEIL - _roll_sv.bounding_box().max.Z
    print(f"  {'servo-top to bay-ceiling gap':58s} {_gap:8.2f} mm   "
          f"{'OK' if _gap >= D.M3_HEAD_H else '** TOO TIGHT **'}")
    ok &= _gap >= D.M3_HEAD_H
    ok &= clearance("buffer: flange bolt heads vs roll servo",
                    F.flange_bolts(), servo_mock_x())
    # carrier WALL screw pan heads reach IN to radius 13.19 - 5.0/2 = 10.69
    # about the roll axis; the yoke_roll pads (r PAD_D/2) spin just inside them
    _pad_r, _head_in = D.PAD_D / 2, 13.19 - D.CASE_HEAD_D / 2
    print(f"  {'yoke pad radius under the wall-screw head reach':58s} "
          f"{_pad_r:8.2f} mm   {'OK' if _pad_r <= _head_in else '** TOO WIDE **'}")
    ok &= _pad_r <= _head_in

    print("== yoke_pitch vs thigh (leg_link + servo), hip -110/+60 (+5 margin) ==")
    # flexion is NEGATIVE here (knee swings toward +x). The full leg_link rides
    # in this sweep: its idler grip plate / web share the idler arm's Y band,
    # and THEY (not the servo) set the mechanical limit.
    thigh_assy = ll + servo_mock_y()
    for ang in (0, -60, -95, -105, -110, -115, 60, 65):
        sv = Rot(0, ang, 0) * thigh_assy
        ok &= check(f"thigh assy at hip {ang:+d} deg", vol(yp, sv))
    # buffer at the ROM extremes, printed+screws only (the servo is excluded:
    # its discs CONTACT the yoke pads by design -- that's the bolted bearing)
    yp_scr = yp + F.disc_screws_y()
    for ang in D.ROM["hip_pitch"]:
        ok &= clearance(f"buffer: hip {ang:+.0f}: yoke+discs vs thigh+grip",
                        yp_scr, Rot(0, ang, 0) * (ll + grip))
    print("== deep flexion vs the stage above (yoke_roll, yaw_carrier, roll servo) ==")
    # v3yaw: the stage above the hip is the carrier (its roll bay), not the pelvis.
    for ang in (-110, -115):
        th = Pos(0, 0, -D.ROLL_TO_PITCH) * Rot(0, ang, 0) * thigh_assy
        ok &= check(f"thigh at hip {ang:+d} vs yoke_roll", vol(yr, th))
        for roll in (0, 25, -25):
            hip_deep = Pos(0, 0, roll_axis) * Rot(roll, 0, 0) * th
            ok &= check(f"thigh at hip {ang:+d} roll {roll:+d} vs carrier",
                        vol(yc, hip_deep))

    print("== foot + ankle servo vs shin link at ankle -40..+40 ==")
    ankle_z = D.FOOT_T - D.FOOT_POCKET_D + 12.36        # 16.36 above sole bottom
    ankle_sv = Pos(0, 0, ankle_z) * Rot(0, 0, 0) * (
        # ankle servo: axis Y, length along X (output end forward +10.11), lying flat
        Rot(0, 90, 0) * servo_mock_y())  # rotate: Z(length) -> X
    for ang in (0, 40, -40):
        sl = Pos(0, 0, ankle_z) * Rot(0, ang, 0) * Pos(0, 0, D.LINK_DROP) * parts.leg_link()
        ok &= check(f"shin link at ankle {ang:+d} vs foot", vol(ft, sl))
        ok &= check(f"shin link at ankle {ang:+d} vs ankle servo", vol(sl, ankle_sv))
    ok &= check("ankle servo vs foot (contact only)", vol(ft, ankle_sv))
    # buffer at the ankle ROM extremes, with every fastener aboard. need=0.35,
    # not SWEEP_BUFFER: the fork horn arm passes 0.40 outside the foot wall by
    # LOCKED design (FOOT_WALL_T comment) -- that band caps the achievable
    # buffer at this joint and 0.5 would flag settled geometry.
    shin_stack = Pos(0, 0, D.LINK_DROP) * (ll + grip) + F.disc_screws_y()
    ft_scr = ft + F.foot_screws()
    for ang in D.ROM["ankle"]:
        # 0.30, down from 0.35: the FRONT tie-down boss (the second front screw,
        # added 2026-07-28) is what now sets this, not the fork horn arm. Its arc
        # sits FOOT_ROTOR_CLEAR_R - PAD_D/2 = 0.30 off the shin pad, and buying
        # more means pushing the arc out until it unseats the screw head. 0.30 is
        # the same running clearance the yaw-carrier bay slip fit already ships
        # at -- accepted deliberately, and measured at both ROM extremes.
        ok &= clearance(f"buffer: ankle {ang:+.0f}: foot+screws vs shin+screws",
                        Pos(0, 0, ankle_z) * Rot(0, ang, 0) * shin_stack,
                        ft_scr, need=0.295)   # measures 0.300000; float headroom

    print("== ankle-servo CABLE connector vs shin fork through ankle ROM ==")
    # WRONG PREMISE, 2026-07-28 (user: "the servo cables connect next to the idler
    # wheel, not at the heel"). This mocked the plug on the HEEL END FACE, which
    # dimensions.py's measured SV_CONN block explicitly rules out -- the sockets
    # open out of the IDLER-SIDE FACE, in the trench band SV_CONN_L (11.75..16.35
    # behind the axis), "never the cable-END face". So this check has been proving
    # clearance in a place the connector never occupies, which is why it did not
    # catch a front boss being put straight over the real one.
    #
    # The heel mock is kept for now because it is still a real swept volume, but
    # it is NOT the connector, and the idler-face plug is NOT yet checked against
    # the shin fork. On the numbers that pair looks tight: the plug protrudes from
    # y -17.35 while the fork's idler plate sits at y -18..-21, both at r ~15 from
    # the ankle axis. Needs the physical part to settle how far the housing stands
    # proud before it can be modelled honestly -- see cad/README.md.
    ankle_conn = Pos(0, 0, ankle_z) * parts.box(-45, -33, -12.36, 12.36, -12.36, 12.36)
    for ang in (0, 40, -40):
        sl = Pos(0, 0, ankle_z) * Rot(0, ang, 0) * Pos(0, 0, D.LINK_DROP) * parts.leg_link()
        ok &= check(f"ankle connector vs shin fork at ankle {ang:+d}", vol(sl, ankle_conn))

    print("== multi-axis worst cases (issue #4: 1-DOF sweeps miss combined poses) ==")
    # Chain in the KNEE frame: thigh leg_link above (origin = knee axis via its
    # lower joint at -LINK_DROP... we place the thigh link so its LOWER joint sits
    # at the origin), shin leg_link rotated by the knee angle, foot + ankle servo
    # rotated by knee THEN ankle. Non-adjacent pairs (thigh vs foot, knee servo vs
    # foot) only close in when BOTH joints fold -- exactly what the 1-DOF sweeps
    # never tried.
    thigh_ll = Pos(0, 0, D.LINK_DROP) * parts.leg_link()   # lower joint at origin
    for k in (-95, -60):
        shin_frame = Rot(0, k, 0)
        for a in (-40, 0, 40):
            foot_frame = shin_frame * Pos(0, 0, -D.LINK_DROP) * Rot(0, a, 0)
            ft2 = foot_frame * Pos(0, 0, -ankle_z) * parts.foot()
            asv2 = foot_frame * Rot(0, 90, 0) * servo_mock_y()
            ok &= check(f"knee {k:+d} ankle {a:+d}: thigh link vs foot",
                        vol(thigh_ll, ft2))
            ok &= check(f"knee {k:+d} ankle {a:+d}: thigh link vs ankle servo",
                        vol(thigh_ll, asv2))
    # hip_pitch + knee: the upper-joint arms (worst-case fork profile from the
    # 1-DOF block) vs the SHIN when both hip and knee fold mid-stride
    for h, k in ((60, -95), (60, -60), (-60, -95), (-60, -60)):
        arms2 = Rot(0, h, 0) * (arm_h + arm_i)
        shin2 = Rot(0, h, 0) * Pos(0, 0, -D.LINK_DROP) * Rot(0, k, 0) * parts.leg_link()
        ok &= check(f"hip {h:+d} knee {k:+d}: upper arms vs shin link",
                    vol(arms2, shin2))

    print("== HEAD STACK: tower top / imu_carrier / gopro_base / IMU board ==")
    # Added 2026-07-27 after a real miss: raising the carrier tongue drove 2.5 mm
    # of material up into gopro_base and this script said ALL CLEAR, because it
    # only ever looked at leg kinematics. Everything above the pelvis was unchecked.
    carrier = parts.imu_carrier()
    gopro = Pos(0, 0, D.IMU_CARRIER_T) * parts.gopro_base()   # seats on the 3 mm pad
    tower_at_top = Pos(0, 0, -D.TOWER_H) * parts.tower()      # carrier z=0 = tower top
    ok &= check("imu_carrier vs gopro_base", vol(carrier, gopro))
    ok &= check("imu_carrier vs tower", vol(carrier, tower_at_top))
    ok &= check("gopro_base vs tower", vol(gopro, tower_at_top))

    # The IMU itself, seated in its pocket.
    _px, _py, _pt = D.IMU_PCB
    imu_pcb = Pos(0, D.IMU_CY, D.IMU_PCB_Z + _pt / 2) * Box(_px, _py, _pt)
    ok &= check("GY-BNO08X seated vs imu_carrier", vol(carrier, imu_pcb))
    ok &= check("GY-BNO08X seated vs gopro_base", vol(imu_pcb, gopro))
    ok &= require("pocket floor under the IMU",
                  vol(carrier, Pos(0, D.IMU_CY, D.IMU_PCB_Z - 0.5)
                      * Box(_px * 0.8, _py * 0.8, 0.8)), 20.0)

    # Both M2.5 pilots must be open, and there must be wall left around them to tap.
    for _sy in (D.IMU_CY + D.IMU_SCREW_DY, D.IMU_CY - D.IMU_SCREW_DY):
        ok &= check(f"M2.5 pilot bore clear at y{_sy:+.1f}",
                    vol(carrier, Pos(D.IMU_SCREW_X, _sy, D.IMU_PCB_Z - 1.0)
                        * Cylinder(D.M25_TAP / 2, 2.0)))
        ok &= require(f"tappable material round pilot y{_sy:+.1f}",
                      vol(carrier, Pos(D.IMU_SCREW_X, _sy, D.IMU_PCB_Z - 1.5)
                          * (Cylinder(2.6, 2.4) - Cylinder(D.M25_TAP / 2, 3.0))), 15.0)

    # Soldered header tails hang below the board along the pad row and must have a
    # clear run out the rear -- a slot that stops short fouls them on insertion.
    ok &= check("pin-tail slot clear along the pad row",
                vol(carrier, Pos(-_px / 2 + 1.27, D.IMU_CY - _py / 2 - 3,
                                 D.IMU_PCB_Z - 1.0) * Box(1.6, _py + 8, 2.0)))

    print("\nALL CLEAR" if ok else "\nINTERFERENCES FOUND — fix before printing")


if __name__ == "__main__":
    main()
