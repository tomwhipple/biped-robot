"""Assembly interference checks: mock servo bodies + printed parts, boolean
intersection volumes at joint-range extremes. All volumes should be ~0 mm^3.

Run:  .venv/bin/python cad/check_assembly.py     (exit 0 = clear, 1 = interference)
"""
import math
import sys

from build123d import *
import dimensions as D
import parts
import fasteners as F


def servo_mock():
    """The ST3215 in ONE canonical frame -- the single source of servo truth.

    Output axis +Y at the origin, horn +Y, body length along Z: output end
    +10.11, cable end -35.11.

    servo_mock_y/_x/_z are rigid ROTATIONS of this, so the three orientations
    cannot drift apart. They HAD drifted (found 2026-07-30): _y was rebuilt to
    the vendor solid on 2026-07-29 but _x and _z still ran the case all the way
    to SV_BOTFACE and cut a O25 disc RECESS -- two things the vendor solid
    disproves. Consequence: every hip part that "seated" on the idler side was
    being checked against 2.60 mm of phantom material, and every part that
    should have cleared the PROUD disc was checked against a well that is not
    there. Rotating one body removes the whole class of bug.

    Measured off cad/vendor/ST3215.step; see the wheel-end block in
    dimensions.py. Corners are square where the real case is radiused -- that
    is deliberate, a slightly oversized mock is the conservative direction for
    an interference check.
    """
    # CASE. Asymmetric about the output axis: the idler side stops 2.60 mm short
    # of the mirrored face. There is NO material at SV_BOTFACE.
    s = parts.box(-D.SV_WID / 2, D.SV_WID / 2,
                  D.SV_IDLER_CASE_FACE, D.SV_TOPFACE,
                  -D.SV_AXIS_FROM_REAR, D.SV_AXIS_FROM_OUT_END)
    # horn-side rib, on the cable half (cable end is -Z in this frame)
    s += parts.box(-D.SV_HORN_RIB_HW, D.SV_HORN_RIB_HW,
                   D.SV_TOPFACE, D.SV_TOPFACE + D.SV_HORN_RIB_H,
                   -D.SV_HORN_RIB_L[1], -D.SV_HORN_RIB_L[0])
    # moulded back-cover platform, proud of the idler case face
    s += parts.box(-D.SV_IDLER_BOSS_HW, D.SV_IDLER_BOSS_HW,
                   D.SV_IDLER_BOSS_Y, D.SV_IDLER_CASE_FACE,
                   D.SV_IDLER_BOSS_Z[0], D.SV_IDLER_BOSS_Z[1])
    # DELIBERATELY NOT MODELLED: the connector trench and the stator screw-boss
    # pockets. Both are HOLES in the idler face, and nothing of ours protrudes
    # into either (connector openings are windows -- absences -- in our parts,
    # and the yaw seat pads are checked against the pelvis, not against this).
    # Cutting them can only make the mock SMALLER than the real servo, which is
    # the unsafe direction for an interference check. Leaving them solid keeps
    # the mock conservative.
    #
    # They were cut here briefly on 2026-07-30 and the vendor comparison caught
    # it: 372 mm3 of real servo went missing. Probing the vendor solid also
    # showed both constants were off the datum they are written against --
    # the trench floor is 5.10 below the case face (SV_CONN_FLOOR says 4.78)
    # and the "1.78 boss recess" is really a sub-1 mm screw hole 3.00 deep
    # inside a shallow ~1.5 pocket. If either is ever needed, MEASURE FIRST.
    #
    # WHEEL ENDS. Both discs stand PROUD of their case face with a clear annular
    # moat, and each carries a free hub proud again. All of it ROTATES with the
    # joint, so a mount that clamps it binds the joint.
    s += parts.cyl_y(D.SV_DISC_R, D.SV_IDLER_FACE, D.SV_IDLER_CASE_FACE, 0, 0)
    s += parts.cyl_y(D.SV_IDLER_HUB_HW, D.SV_BOTFACE, D.SV_IDLER_FACE, 0, 0)
    s += parts.cyl_y(D.SV_BOSS_D / 2, D.SV_TOPFACE, D.SV_HORN_FACE, 0, 0)
    return s


def servo_mock_y():
    """Leg pitch joints (hip pitch, knee) and -- rolled 90 deg -- the ankle.
    Output axis +Y, body length along Z, output end +10.11, cable end -35.11."""
    return servo_mock()


def servo_mock_x():
    """Roll servo: output axis +X (horn +X), body length along Z with the
    output end DOWN -- cable end +35.11 at the top, output end -10.11."""
    return Rot(0, 180, 0) * Rot(0, 0, 90) * servo_mock()


def servo_mock_z():
    """Yaw servo (v3yaw): output axis VERTICAL with the horn DOWN. Case length
    along X (YAW_CASE_X_REAR..FRONT), width along Y, idler face UP."""
    return Rot(0, 90, 0) * Rot(0, 0, -90) * servo_mock()


# ------------------------------------------------------------- torso mocks
# The bought parts the v6 torso holds: the driver board, its component side, and
# the camera. They live HERE rather than in export_assembly because this module
# is the one that may not import that one (export_assembly imports this), and a
# mock that exists twice is a mock that drifts. export_assembly delegates.
def board_pcb_mock():
    """The General Driver PCB seated in the v6 recess, pelvis frame. CORRECTED
    outline 65.01 x 56.01 (the wiki's 65 x 65 was 8.99 mm of board that does not
    exist -- BOARD_GD_OUTLINE), standing upright and transverse with its 65.01
    PORTS EDGE UP: forward face on the standoffs at BR_PCB_X0, y +/-32.505,
    z BR_BOT_Z..BR_TOP_Z, so only the top 15 mm clears the deck.

    ITS OWN MOUNTING HOLES ARE MODELLED (BOARD_GD_HOLE_D on the BOARD_GD_HOLES
    grid). Without them a solid slab reads as blocking its own screws -- the
    kind of false positive that gets explained away in a comment once and then
    hides a real one later."""
    hy = D.BOARD_GD_OUTLINE[0] / 2
    p = parts.box(D.BR_PCB_X1, D.BR_PCB_X0, -hy, hy, D.BR_BOT_Z, D.BR_TOP_Z)
    for sy in (D.BOARD_GD_SCREW_DY, -D.BOARD_GD_SCREW_DY):
        for sz in (D.BR_SCREW_Z, D.BR_UPPER_ROW_Z):
            p -= parts.cyl_x(D.BOARD_GD_HOLE_D / 2, D.BR_PCB_X1 - 1,
                             D.BR_PCB_X0 + 1, sy, sz)
    return p


def board_comp_mock():
    """Everything fitted to the board's AFT face, as ONE conservative slab of
    BOARD_GD_COMP -- the 40-pin header is the tall one, and the recess cheeks
    are sized against it (BR_RIM_PROUD)."""
    hy = D.BOARD_GD_OUTLINE[0] / 2
    return parts.box(D.BR_COMP_X, D.BR_PCB_X1, -hy, hy, D.BR_BOT_Z, D.BR_TOP_Z)


def camera_mock():
    """GoPro MAX 360 stand-in in GOPRO_BASE's frame (z=0 at the base's bottom,
    i.e. the deck top): body box plus the two folded mount fingers reaching down
    into the base's prong slots, body bottom ~6 above the M5 hole centre."""
    # Body bottom CORRECTED 2026-08-04 (downstream pass): it was hole_z + 6.0,
    # inherited from the v3 tower-era mock. The prongs' round tops reach
    # hole_z + GP_PRONG_OD/2 = hole_z + 7.5 about the M5 axis, so a body at +6
    # swallowed the top 1.5 mm of all three prongs -- 82.77 mm3, and it is the
    # mock that is wrong, not the mount: on the real camera the prongs nest in
    # the gap BETWEEN its two fingers, which is what the +7.5 clearance is. The
    # 0.5 on top is the running gap the folding mount needs to swing shut.
    hole_z = D.GP_BASE_T + D.GP_HOLE_H
    bot = hole_z + D.GP_PRONG_OD / 2 + 0.5
    dx, dy, dz = D.CAM_BODY
    body = parts.box(-dx / 2, dx / 2, -dy / 2, dy / 2, bot, bot + dz)
    fingers = None
    for cy in (-(D.GP_PRONG_T + D.GP_SLOT) / 2, (D.GP_PRONG_T + D.GP_SLOT) / 2):
        f = parts.box(-6, 6, cy - 1.45, cy + 1.45, hole_z - 6, bot + 1)
        fingers = f if fingers is None else fingers + f
    return body + fingers


def _check_mock_frames():
    """Assert each rotation lands where its docstring says. Cheap insurance:
    a wrong Euler triple would silently check every part against a servo lying
    in the wrong direction, and every volume would still read 0.00."""
    exp = {
        "y": (-D.SV_WID / 2, D.SV_WID / 2, D.SV_BOTFACE, D.SV_HORN_FACE,
              -D.SV_AXIS_FROM_REAR, D.SV_AXIS_FROM_OUT_END),
        "x": (D.SV_BOTFACE, D.SV_HORN_FACE, -D.SV_WID / 2, D.SV_WID / 2,
              -D.SV_AXIS_FROM_OUT_END, D.SV_AXIS_FROM_REAR),
        "z": (D.YAW_CASE_X_REAR, D.YAW_CASE_X_FRONT, -D.SV_WID / 2, D.SV_WID / 2,
              -D.SV_HORN_FACE, -D.SV_BOTFACE),
    }
    for tag, fn in (("y", servo_mock_y), ("x", servo_mock_x), ("z", servo_mock_z)):
        b = fn().bounding_box()
        got = (b.min.X, b.max.X, b.min.Y, b.max.Y, b.min.Z, b.max.Z)
        for g, e in zip(got, exp[tag]):
            assert abs(g - e) < 1e-6, (
                f"servo_mock_{tag} frame moved: {tuple(round(v, 2) for v in got)} "
                f"!= {exp[tag]}")


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


# ------------------------------------------------------------- wheel ends
def _cyl(axis, r, a0, a1, u, v):
    return {"x": parts.cyl_x, "y": parts.cyl_y, "z": parts.cyl_z}[axis](
        r, a0, a1, u, v)


def unsupported(part, axis, face, sign, ctr, head_d, t=0.4):
    """Volume of a seated head/washer annulus that hangs over air.

    The screw checks elsewhere prove the head is SWALLOWED (no interference)
    and that the driver can REACH it. Neither says the head has anything to
    PULL AGAINST: a head whose land runs off the rim of its pad is a clamp
    with nothing under half of it. Inner radius is 2.6, outside the teardrop
    peak of the PAD_HOLE bore, so the bore void never counts as unsupported."""
    r = head_d / 2
    a0, a1 = (face - t, face) if sign > 0 else (face, face + t)
    ring = (_cyl(axis, r, a0, a1, *ctr)
            - _cyl(axis, 2.6, a0 - 1, a1 + 1, *ctr))
    return ring.volume - vol(part, ring)


def engage(label, length, stack, washer=0.0, depth=None):
    """Thread engagement of a disc screw: does it reach, and does it bottom?

    The disc screws thread into the servo's own disc, and that disc is a thin
    FLANGE at the O14 bolt circle -- D.DISC_THREAD (2.1 mm), not the 3.35 mm
    the vendor solid's disc body suggests (the 3.1 mm hub is inboard of the
    screws). Too long and the screw hits the bottom of the hole and jacks the
    joint apart instead of clamping it; too short and there is no thread to
    hold. Neither shows up as an interference, so no boolean check can see it."""
    depth = D.DISC_THREAD if depth is None else depth
    eng = length - stack - washer
    if eng > depth:
        state = f"** BOTTOMS OUT {eng - depth:.2f} early **"
    elif eng < D.DISC_THREAD_MIN_ENGAGE:
        state = f"** ONLY {eng:.2f} OF THREAD **"
    else:
        state = "OK"
    print(f"  {label:58s} {eng:8.2f} mm   {state}")
    return D.DISC_THREAD_MIN_ENGAGE <= eng <= depth



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

    print("== hip yokes + thigh servo vs yaw_carrier + roll servo, roll -55..+55 ==")
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
    for ang in (0, 25, -25, 55, -55):   # +-55: the 2026-08-03 abduction widening
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
    # ...and does that bay actually TOUCH the servo on the idler side? Clearing
    # it is not the same as seating on it: until 2026-08-01 the rear wall stood
    # at the MIRRORED SV_TOPFACE and cleared the servo by 2.60 mm everywhere, so
    # the check above passed while both retention screws pulled on air. Probe the
    # side lands -- outboard of the platform detent channel, where the seat is
    # supposed to land on the real case face.
    _rsv = Pos(0, 0, D.CARRIER_ROLL_AXIS) * servo_mock_x()
    for _y in (11.8, -11.8):
        _probe = parts.cyl_x(0.8, -25, 0, _y, D.CARRIER_ROLL_AXIS + 23.0)
        _car = yc & _probe
        _svm = _rsv & _probe
        _inner = max((s.bounding_box().max.X for s in _car.solids()), default=None)
        _first = min((s.bounding_box().min.X for s in _svm.solids()), default=None)
        _gap = None if (_inner is None or _first is None) else _first - _inner
        _state = "OK" if _gap is not None and _gap <= D.GRIP_SEAT_CLR + 0.05 \
            else "** SEAT STANDS OFF THE CASE **"
        print(f"  {'carrier idler seat bears on the case (y%+.1f)' % _y:58s} "
              f"{_gap if _gap is not None else float('nan'):8.2f} mm   {_state}")
        ok &= _state == "OK"
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

    print("== WHEEL ENDS: does each disc screw actually get thread? ==")
    # BENCH 2026-07-30 (user): "the idler disc is 2.1 mm thick... though there's
    # a central hub that's 3.1". Sectioning the vendor solid by radius agrees --
    # hub 3.23-3.35 to r 4.5, flange 2.20 beyond -- and the O14 bolt circle is
    # at r 7.00, i.e. in the FLANGE. Everything the BOM specced for these holes
    # was sized against the 3.35 slab and drives straight through it.
    _pitch_stack = abs(D.IDLER_ARM_INNER - D.SV_IDLER_FACE) + D.PLATE   # 3.60
    _roll_stack = (abs(D.ROLL_ARM_INNER - D.SV_IDLER_FACE) + D.PLATE
                   - D.ROLL_IDLER_PAD_SINK)                             # 5.80
    ok &= engage("horn discs: M3x5 into the horn flange (was M3x6)",
                 5.0, D.PLATE, depth=D.HORN_THREAD)
    ok &= engage("pitch idlers: same screw, no washer (yoke_pitch + fork)",
                 D.DISC_SCREW_THREAD, _pitch_stack)
    # yoke_roll used to be DEFERRED at a 7.15 stack that fits no stock screw.
    # FIXED 2026-07-30 (user: "the yoke roll pad is too thick for our screws"):
    # the yaw_carrier bay wall floors this stack at 6.15 no matter how thin the
    # arm gets, so it takes the longer M3x8 into a pad sunk 1.35. See the ROLL_*
    # block in dimensions.py for the full budget.
    ok &= engage("roll idler: M3x8 into a pad sunk 1.35 (was DEFERRED)",
                 D.ROLL_DISC_SCREW_THREAD, _roll_stack)
    # ...and the head has to have something to pull against once it gets there.
    for _lbl, _part, _ax, _face, _sgn, _o in (
            ("yoke_pitch idler pad", yp, "y", D.IDLER_ARM_INNER - D.PLATE, -1, (0, 0)),
            ("leg_link fork idler pad", ll, "y", D.IDLER_ARM_INNER - D.PLATE, -1,
             (0, -D.LINK_DROP)),
            # the sunk roll pad: the head now lands 1.35 below the arm face, so
            # prove it still has a full annulus of pad under it (r 9.85 vs the
            # O20 rim) and is not hanging over the recess wall.
            ("yoke_roll idler pad (sunk)", yr, "x",
             D.ROLL_ARM_INNER - D.PLATE + D.ROLL_IDLER_PAD_SINK, -1, (0, 0))):
        _worst = max(unsupported(_part, _ax, _face, _sgn,
                                 (_o[0] + dx, _o[1] + dz), D.M3_HEAD_D)
                     for dx, dz in ((7, 0), (-7, 0), (0, 7), (0, -7)))
        ok &= check(f"{_lbl}: M3 head land fully supported", _worst)

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
    ok &= check("foot vs its 3 tab screws", vol(ft, F.foot_screws()))
    # v6 (2026-08-04): there are no torso PARTS left to bolt on but one. The
    # tray and the board frame folded into pelvis(), taking nine screws with
    # them; what is left is the board's two M2.5 from aft and gopro_base's four
    # M3 from above. Everything is drawn in the pelvis frame.
    gp = Pos(D.GP_MOUNT_X, 0, 0) * parts.gopro_base()
    _pcb_v6, _comp_v6 = board_pcb_mock(), board_comp_mock()
    _camera_at = lambda: Pos(D.GP_MOUNT_X, 0, 0) * camera_mock()
    gp_scr, brd_scr = F.gopro_screws(), F.board_screws()
    ok &= check("gopro_base vs its 4 M3", vol(gp, gp_scr))
    ok &= check("pelvis vs the gopro screws (into the deck bosses)",
                vol(pv, gp_scr))
    ok &= check("pelvis vs the 2 board screws (into the web standoffs)",
                vol(pv, brd_scr))
    # The deck top is still a printed-on-bed face and the belt still runs across
    # it, so nothing may stand in the belt's band. In v6 that band has to clear
    # the stator heads (flush) -- the four tray screws that used to sit proud
    # here went with the tray.
    _belt = parts.box(D.BT_BELT_X[0], D.BT_BELT_X[1],
                      -D.BT_BELT_SLOT_Y[1], D.BT_BELT_SLOT_Y[1], 0, 2.0)
    ok &= check("belt band on the deck top vs the stator screw heads",
                vol(_belt, F.deck_stator_screws()))
    ok &= check("belt band vs gopro_base + its screws", vol(_belt, gp + gp_scr))

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
    # that arrives in §7c, and the frame's own mount screws clear of the driver
    # board -- the frame is bolted to the housing empty, then the board goes on.
    for label, host, group in (
            ("leg_link: 6 grip screws", ll, "screws_grip"),
            ("foot: 3 tab screws", ft, "screws_foot"),
            ("pelvis: 8 stator screws down through the deck", pv, "screws_deck"),
            ("yaw_carrier: 8 wall screws", yc, "screws_yaw_wall"),
            ("yaw_carrier: 4 horn bolts (bay still empty, §7b)", yc,
             "screws_yaw_horn"),
            ("yoke_pitch: 8 disc screws", yp, "screws_disc_y"),
            ("leg_link fork: the same 8 disc screws one joint down",
             Pos(0, 0, D.LINK_DROP) * ll, "screws_disc_y"),
            ("yoke_roll: 8 disc screws", yr, "screws_disc_x"),
            ("yoke_roll: 4 flange bolts", yr, "screws_flange"),
            # v5 torso. The two aft groups both come in along -x from behind:
            # first the frame's four M3 into the housing ribs, then -- once the
            # board is on its standoffs -- the four M2.5 whose heads land on the
            # PCB's aft face in open air, which is the whole reason the board
            # faces aft (the tower's four ended 3.17 mm from a wall).
            ("gopro_base: 4 M3 down into the deck bosses", gp,
             "screws_gopro"),
            ("board: 2 M2.5 into the web standoffs, from aft", pv,
             "screws_board")):
        ok &= check(f"path: {label}", vol(host, F.paths(group)))
    # ...and the paths that cross PART BOUNDARIES, which is where the tightest
    # access numbers live. The eight stator screws are driven vertically with
    # the BOARD and the GOPRO BASE already installed -- that is the state the
    # robot spends its life in, and a re-torque has to be possible in it. The
    # board is the near miss: its forward face stands at BR_PCB_X0 (-42.01) and
    # the -32.75 row's ACCESS_D cylinder reaches back to -36.25, so 5.76 mm.
    #
    # The CAMERA is deliberately NOT in this set. Its body is 64 wide and hangs
    # over the stator rows from z 19.5 up, so it does foul those cylinders --
    # measured below and reported, not gated, because the camera is a click-on
    # accessory held by one thumbscrew: it is not fitted when the servos are.
    for _lbl, _host in (("board + components", _pcb_v6 + _comp_v6),
                        ("gopro_base", gp),
                        ("battery bay walls", pv)):
        ok &= check(f"path: stator screws vs {_lbl}",
                    vol(_host, F.paths("screws_deck")))
    _cam_foul = vol(_camera_at(), F.paths("screws_deck"))
    print(f"  {'note: stator paths vs the CAMERA (unclip it first)':58s} "
          f"{_cam_foul:8.2f} mm3  {'clear' if _cam_foul <= 1.0 else 'FOULS'}"
          f" -- accessory, not gated")
    # The board's own Ø3 holes are in the mock, so the two M2.5 must pass
    # through them cleanly -- shank in the hole, head landing on the aft face.
    ok &= check("board screws pass their own holes in the PCB",
                vol(board_pcb_mock(), brd_scr))
    ok &= check("path: gopro screws vs pelvis (down through the deck)",
                vol(pv, F.paths("screws_gopro")))

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
    # ...and the ankle servo. BENCH-MEASURED 2026-07-30 (user: "it no longer
    # fits in the foot"): the rib DOES ride the +Y tab, settling the
    # vendor-vs-dims dispute the old note here deferred -- the vendor solid's
    # 0.2 mm "clearance" was its known ~0.2 idler-datum error. parts.foot now
    # carries an open-top rib window (and gave up the z 26.61 horn screw whose
    # bearing land the window consumes), so this is a hard check like every
    # other socket: the drop-in must come through clean.
    _ankle_sv = Pos(0, 0, D.FOOT_T - D.FOOT_POCKET_D + 12.36) \
        * Rot(0, 90, 0) * servo_mock_y()        # as placed below, lying flat
    _rib_vol, _at = insert_scan(ft, _ankle_sv, *SERVO_INSERT["foot"])
    ok &= check("foot: ankle servo drop-in (through the rib window)", _rib_vol)

    print("== GRIP-SCREW HEADS vs the arms that sweep them (the bench skew) ==")
    # 2026-07-28 bench find: the thigh links SKEWED on their servos. Cause: a
    # proud pan head (O5.0 x 2.0) on the horn grip plate sits in the 0.70 mm
    # band the yoke/fork arm above sweeps through -- overlap reached 25.5 mm3
    # from hip +/-60 deg on, so the arm rode the heads and pried the link
    # sideways. Fix: 90-deg countersinks + M2.5 FLAT-head self-tappers, heads
    # flush; these checks hold the whole ROM plus the SWEEP_BUFFER.
    lo, hi = D.ROM["hip_pitch"]
    for ang in (lo, -95, -60, 0, 60, hi):
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

    print("== yoke_pitch vs thigh (leg_link + servo), hip -110/+90 (+5 margin) ==")
    # flexion is NEGATIVE here (knee swings toward +x). The full leg_link rides
    # in this sweep: its idler grip plate / web share the idler arm's Y band,
    # and THEY (not the servo) set the mechanical limit.
    thigh_assy = ll + servo_mock_y()
    for ang in (0, -60, -95, -105, -110, -115, 60, 90, 95):
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
        for roll in (0, 25, -25, 55, -55):
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
    # knee angles here are PHYSICAL (+y rotation) and POSITIVE is flexion --
    # the shank folding backward. They were -95/-60 until 2026-08-02, when the
    # plant's knee axis was corrected to "0 -1 0" (docs/servo-map.md); the old
    # values were folding the knee the bird way, which is not the pose the
    # robot ever holds. See dimensions.ROM["knee"].
    for k in (95, 60):
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
    for h, k in ((60, 95), (60, 60), (-60, 95), (-60, 60)):
        arms2 = Rot(0, h, 0) * (arm_h + arm_i)
        shin2 = Rot(0, h, 0) * Pos(0, 0, -D.LINK_DROP) * Rot(0, k, 0) * parts.leg_link()
        ok &= check(f"hip {h:+d} knee {k:+d}: upper arms vs shin link",
                    vol(arms2, shin2))

    print("== TORSO v6: one printed torso + the camera on the roof ==")
    # v6 folded the tray and the board frame into pelvis(), so most of what the
    # v5 section checked is now checked by the pelvis's own booleans elsewhere.
    # What is left here is what a single-part torso still cannot prove to
    # itself -- the things it HOLDS, and how they get in and out:
    #
    #   (a) THE FLOOR still governs. Every torso solid, printed or bought, stays
    #       above TORSO_FLOOR_Z, 1.0 mm over the yaw carrier's horn plate. The
    #       carrier is the highest moving part of the leg, so that plane is the
    #       whole leg-clearance argument, and it is checked as a number and then
    #       the hard way against the swept carrier stacks.
    #   (b) THE BOARD goes in DOWNWARD between the recess cheeks and is held by
    #       two screws at its lower row only -- the upper row lands above the
    #       deck, where a deck-top-down print cannot put a boss. So the drop-in
    #       path is a real check, not a formality.
    #   (c) THE PACK rests in a V of two 47 deg chamfers with no floor under it,
    #       and lifts straight out through the deck aperture.
    #   (d) THE CAMERA is back. It is 154 g on a 30 x 24 pad, so where it can
    #       reach matters -- and what it blocks (see the stator-path note above).
    _ysv_both = ysv + Pos(0, -D.HIP_SEP / 2, YAW_MID) * servo_mock_z()
    _cam = _camera_at()
    for _lbl, _s in (("pelvis (the whole torso)", pv),
                     ("driver board + components", _pcb_v6 + _comp_v6),
                     ("board screws", brd_scr),
                     ("gopro_base", gp), ("gopro screws", gp_scr)):
        _zmin = _s.bounding_box().min.Z
        _gap = _zmin - D.TORSO_FLOOR_Z
        print(f"  {'floor: ' + _lbl + ' lowest z':58s} {_zmin:8.2f} mm   "
              f"{'OK' if _gap >= 0 else '** BELOW TORSO_FLOOR **'} "
              f"({_gap:+.2f} vs {D.TORSO_FLOOR_Z})")
        ok &= _gap >= 0

    ok &= check("gopro_base seated on the deck vs pelvis", vol(pv, gp))
    ok &= check("camera seated vs gopro_base (fingers in the slots)",
                vol(gp, _cam))
    ok &= check("camera vs pelvis (it overhangs the deck)", vol(pv, _cam))
    ok &= check("gopro_base + camera vs both yaw servos",
                vol(gp + _cam, _ysv_both))
    ok &= check("board + components vs both yaw servos",
                vol(_pcb_v6 + _comp_v6, _ysv_both))
    # ...and the hard way, against the carriers through their whole yaw sweep.
    for _yaw in (0, D.YAW_SWEEP, -D.YAW_SWEEP):
        _st = yaw_stack(D.HIP_SEP / 2, _yaw) + yaw_stack(-D.HIP_SEP / 2, _yaw)
        ok &= check(f"pelvis vs carrier stacks at yaw {_yaw:+.0f}", vol(pv, _st))
        ok &= check(f"board + components vs carrier stacks at yaw {_yaw:+.0f}",
                    vol(_pcb_v6 + _comp_v6, _st))
        ok &= check(f"gopro + camera vs carrier stacks at yaw {_yaw:+.0f}",
                    vol(gp + _cam, _st))
    ok &= clearance("buffer: board vs carrier stack at yaw -45",
                    _pcb_v6 + _comp_v6, yaw_stack(D.HIP_SEP / 2, -D.YAW_SWEEP))

    # --- HEAT-SET METAL. v6 has ONE insert pattern left: the four under the
    # gopro pad. (The tray's four and the rear-web ribs' four went with the
    # parts they held.) A M3 insert cannot live in a 5 mm deck, which is why
    # each hangs in a GP_BOSS_D boss under it -- an empty ring here is the
    # failure that pattern exists to prevent.
    def _ring_z(x, y, z0, z1, r_out):
        return (parts.cyl_z(r_out, z0, z1, x, y)
                - parts.cyl_z(D.HEATSET_D / 2, z0 - 1, z1 + 1, x, y))

    _gx, _gy = D.GP_SCREW_XY
    for _sx in (D.GP_MOUNT_X + _gx, D.GP_MOUNT_X - _gx):
        for _sy in (_gy, -_gy):
            ok &= require(
                f"deck+boss metal round the gopro heat-set ({_sx:+.0f},{_sy:+.0f})",
                vol(pv, _ring_z(_sx, _sy, -D.DECK_T - D.GP_BOSS_H, 0.0,
                                D.GP_BOSS_D / 2)), 250.0)

    # --- BATTERY. No floor under it any more: it rests in the V of the two
    # 47 deg seat chamfers, which CENTRE it between the bay walls. So the mock
    # is placed where the V actually puts it, from the chamfer geometry -- not
    # against either wall. (BT_SEAT_DROP's derivation was corrected this pass;
    # at the old value the pack floated 0.52 mm above its seat and every check
    # below would have read zero by luck rather than by contact.)
    def _seated(w, ly, h):
        xc = (D.BT_SEAT_X + D.BT_WALL_X1) / 2
        z0 = D.BT_SEAT_Z - ((D.BT_SEAT_BAY_W - w) / 2
                            * math.tan(math.radians(D.BT_SEAT_DEG)))
        return parts.box(xc - w / 2, xc + w / 2, -ly / 2, ly / 2, z0, z0 + h)

    _pack = _seated(*[D.BATT_PACK[i] for i in (1, 0, 2)])
    _env = _seated(*[D.BATT[i] for i in (1, 0, 2)])
    ok &= check("battery pack seated in the chamfer V vs pelvis", vol(pv, _pack))
    ok &= check("battery ENVELOPE seated in the V vs pelvis", vol(pv, _env))
    # ...and it must actually LAND on the chamfers, not hang in the aperture:
    # drop each 0.4 mm further and it has to bite. This is the check that the
    # corrected BT_SEAT_DROP makes meaningful.
    for _lbl, _sol in (("pack", _pack), ("ENVELOPE", _env)):
        _b = _sol.bounding_box()
        _low = parts.box(_b.min.X, _b.max.X, _b.min.Y, _b.max.Y,
                         _b.min.Z - 0.4, _b.max.Z - 0.4)
        ok &= require(f"battery {_lbl} lands on the seat chamfers (0.4 lower)",
                      vol(pv, _low), 1.0)
    for _lbl, _sol in (("pack", _pack), ("ENVELOPE", _env)):
        _b = _sol.bounding_box()
        _lift = parts.box(_b.min.X, _b.max.X, _b.min.Y, _b.max.Y,
                          _b.min.Z, _b.max.Z + 60)
        ok &= check(f"battery {_lbl} lift-out (+60 up) through the aperture",
                    vol(pv, _lift))
        ok &= check(f"battery {_lbl} lift-out vs gopro_base + camera",
                    vol(gp + _cam, _lift))

    # --- DRIVER BOARD: seated, and -- the v6 question -- DROPPED IN FROM ABOVE
    # between the recess cheeks, which is both how it goes in and how it comes
    # out for service.
    ok &= check("driver board PCB seated vs pelvis", vol(pv, _pcb_v6))
    ok &= check("board component envelope vs pelvis (cheeks, ears, web)",
                vol(pv, _comp_v6))
    ok &= check("board + components vs gopro_base + camera",
                vol(gp + _cam, _pcb_v6 + _comp_v6))
    _w, _at = insert_scan(pv, _pcb_v6 + _comp_v6, (0, 0, 1), 70.0)
    print(f"  {'board drops in from above (swept +z 70 mm)':58s} {_w:8.2f} mm3  "
          f"{'OK' if _w <= 1.0 else '** BLOCKED **'} (worst at {_at:+.1f} mm)")
    ok &= _w <= 1.0
    # ...the cheeks have to actually CAPTURE it -- a recess whose cheeks had
    # drifted outboard would pass every collision check above and hold nothing.
    for _s in (1, -1):
        _grip = parts.box(D.BR_PCB_X1, D.BR_PCB_X0,
                          _s * D.BR_CHEEK_Y_IN, _s * (D.BR_CHEEK_Y_IN + 1.5),
                          D.BR_BOT_Z, -D.DECK_T)
        ok &= require(f"recess cheek beside the board's {_s:+.0f}y edge",
                      vol(pv, _grip), 50.0)
    # ...and the two standoffs have to have tappable metal round their pilots.
    for _sy in (D.BOARD_GD_SCREW_DY, -D.BOARD_GD_SCREW_DY):
        _boss = (parts.cyl_x(3.5, D.BR_PCB_X0, D.YAW_BOX_X_REAR, _sy,
                             D.BR_SCREW_Z)
                 - parts.cyl_x(D.M25_TAP / 2, D.BR_PCB_X0 - 1,
                               D.YAW_BOX_X_REAR + 1, _sy, D.BR_SCREW_Z))
        ok &= require(f"standoff boss metal at (y{_sy:+.0f}, z{D.BR_SCREW_Z:.1f})",
                      vol(pv, _boss), 100.0)
    # ...the upper screw row is UNBOLTED BY DESIGN. Assert the reason, so that a
    # future board or a lower BR_BOT_Z that brings it under the deck fails here
    # instead of silently shipping a two-screw mount that could have had four.
    print(f"  {'board upper screw row (unbolted -- must be above deck)':58s} "
          f"{D.BR_UPPER_ROW_Z:8.2f} mm   "
          f"{'OK' if D.BR_UPPER_ROW_Z > 0 else '** BELOW DECK: use 4 screws **'}")
    ok &= D.BR_UPPER_ROW_Z > 0

    # --- SERVICE ACCESS. Same two claims v5 made, re-measured against a torso
    # that now has a board standing up through the deck plane and a camera on
    # the roof: full plug room over each yaw connector trench, and a clear
    # deck-top corridor between the stator screws' driver cylinders for the
    # leads to run aft in.
    # The camera is excluded here for the same reason it is excluded from the
    # stator paths, and it is the same 154 g body doing it: 64 mm across and
    # hanging from z 19.5 up, it shadows the outboard half of both trenches
    # above that height. Measured and reported below. Unclipping it is one
    # thumbscrew, and it is not fitted while anyone is plugging servo leads in.
    for _by in (D.HIP_SEP / 2, -D.HIP_SEP / 2):
        _probe = parts.box(-D.SV_CONN_L[1] - 1.0, -D.SV_CONN_L[0] + 0.6,
                           _by - 10.5, _by + 10.5, 0.0, 40.0)
        ok &= check(f"trench headroom at y {_by:+.0f} vs board + gopro_base",
                    vol(_pcb_v6 + _comp_v6, _probe) + vol(gp, _probe))
        _cf = vol(_cam, _probe)
        print(f"  {'note: trench y %+.0f vs the CAMERA (unclip it first)' % _by:58s} "
              f"{_cf:8.2f} mm3  {'clear' if _cf <= 1.0 else 'SHADOWED'}"
              f" -- accessory, not gated")
    _corr_y = (D.HIP_SEP / 2 - D.CASE_HOLE_LAT + D.ACCESS_D / 2,
               D.HIP_SEP / 2 + D.CASE_HOLE_LAT - D.ACCESS_D / 2)
    for _s in (1, -1):
        _corridor = parts.box(D.DECK_CUT_X, -D.SV_CONN_L[0],
                              _s * _corr_y[0], _s * _corr_y[1], 0.0, 6.0)
        ok &= check(f"deck-top lead corridor (y {_corr_y[0]:.2f}.."
                    f"{_corr_y[1]:.2f}) vs stator drivers",
                    vol(_corridor, F.paths("screws_deck")))
        ok &= check(f"deck-top lead corridor vs gopro_base",
                    vol(_corridor, gp))
    print(f"  {'corridor width between the stator driver cylinders':58s} "
          f"{_corr_y[1] - _corr_y[0]:8.2f} mm   "
          f"{'OK' if _corr_y[1] - _corr_y[0] >= D.WIRE_PLUG_W else '** TOO NARROW **'}")
    ok &= _corr_y[1] - _corr_y[0] >= D.WIRE_PLUG_W

    print("\nALL CLEAR" if ok else "\nINTERFERENCES FOUND — fix before printing")
    # NOTE: this file checks POSE EXTREMES and a handful of combined poses. The
    # swept check -- every joint walked across its whole travel, in the real
    # articulated assembly -- lives in freecad_rom_collide.py, which needs
    # freecadcmd rather than this venv. cad/run_checks.sh runs both.
    return ok


if __name__ == "__main__":
    sys.exit(0 if main() else 1)
