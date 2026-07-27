"""Assembly interference checks: mock servo bodies + printed parts, boolean
intersection volumes at joint-range extremes. All volumes should be ~0 mm^3.

Run:  .venv/bin/python cad/check_assembly.py
"""
from build123d import *
import dimensions as D
import parts


def servo_mock_y():
    """Servo with output axis along +Y at origin (leg pitch joints).
    Body length along Z: top end +10.11, bottom -35.11; horn cylinder and the
    O25 x 0.55 idler recess (which our idler bosses reach into) included."""
    case = parts.box(-12.36, 12.36, D.SV_BOTFACE, D.SV_TOPFACE,
                     -D.SV_AXIS_FROM_REAR, D.SV_AXIS_FROM_OUT_END)
    case -= parts.cyl_y(D.SV_IDLER_RECESS_D / 2, D.SV_BOTFACE - 0.01,
                        D.SV_IDLER_FACE, 0, 0)
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
    case += parts.cyl_x(3.05, D.SV_BOTFACE - D.SV_IDLER_HUB_PROUD,
                        D.SV_IDLER_FACE, 0, 0)
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
    case += parts.cyl_z(3.05, 16.80, D.SV_TOPFACE + D.SV_IDLER_HUB_PROUD, 0, 0)
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

    print("== servo CASE-SCREW HEADS vs the yokes that sit beside them ==")
    # The servo's 6 case-grip screws stand 1.65 mm proud of its case faces at
    # radius hypot(10.25, 8.30) = 13.19 -- so their heads reach IN to radius 10.34.
    # Anything centred on the joint axis and wider than that fouls them. PAD_D was
    # 24 (radius 12) and buried 1.66 mm into them: 18.92 mm3 on yoke_pitch's drive
    # side, 7.45 mm3 on yoke_roll's. Found on the bench 2026-07-27, not here.
    #
    # Only the SIX FITTED screws are modelled (leg_link's grips: both horn-face
    # rows, plus idler-face row CASE_HOLES_BOT[1]) -- the servo has other case
    # holes that take no screw, and counting those invents clashes. leg_link
    # itself is excluded: the screws pass THROUGH it, so its heads sit on its
    # outer face by design.
    _HD, _HH = 5.7, 1.65
    def _case_heads(axis):
        h = None
        for row in D.CASE_HOLES_TOP:                       # horn face, heads out
            for lat in (D.CASE_HOLE_LAT, -D.CASE_HOLE_LAT):
                c = (Pos(lat, D.SV_HORN_FACE + _HH/2, -row) * Rot(90, 0, 0) if axis == "y"
                     else Pos(D.SV_HORN_FACE + _HH/2, lat, row) * Rot(0, 90, 0)) \
                    * Cylinder(_HD/2, _HH)
                h = c if h is None else h + c
        for lat in (D.CASE_HOLE_LAT, -D.CASE_HOLE_LAT):    # idler face, one row
            c = (Pos(lat, D.SV_IDLER_FACE - _HH/2, -D.CASE_HOLES_BOT[1]) * Rot(90, 0, 0)
                 if axis == "y"
                 else Pos(D.SV_IDLER_FACE - _HH/2, lat, D.CASE_HOLES_BOT[1]) * Rot(0, 90, 0)) \
                * Cylinder(_HD/2, _HH)
            h = h + c
        return h
    ok &= check("yoke_pitch vs the 6 fitted case-screw heads",
                vol(parts.yoke_pitch(), _case_heads("y")))
    ok &= check("yoke_roll vs the 6 fitted case-screw heads",
                vol(parts.yoke_roll(), _case_heads("x")))
    _pad_r, _head_in = D.PAD_D / 2, 13.19 - _HD / 2
    print(f"  {'pad radius under the case-screw head reach':58s} {_pad_r:8.2f} mm   "
          f"{'OK' if _pad_r <= _head_in else '** TOO WIDE **'}")
    ok &= _pad_r <= _head_in

    print("== yaw-horn SCREW HEADS vs the roll servo inside the bay ==")
    # The 4x yaw-horn bolts pass UP through the carrier's mount plate into the horn
    # disc, so their HEADS sit proud of the bay ceiling -- pointing straight at the
    # roll servo's rear face. Before CARRIER_ROLL_HEAD_CLEAR the servo was flush
    # against that ceiling (0.00 mm gap) and the heads buried 168 mm3 into it.
    # Nothing caught it: every other check here compares PRINTED PARTS, and a
    # fastener that only exists as a symbol in the drawings is invisible to them.
    _HEAD_D, _HEAD_H = 5.7, 1.65                       # M3 button head
    _roll_sv = Pos(0, 0, D.CARRIER_ROLL_AXIS) * servo_mock_x()
    _heads = None
    for _dx, _dy in ((D.BCD/2, 0), (-D.BCD/2, 0), (0, D.BCD/2), (0, -D.BCD/2)):
        _h = Pos(_dx, _dy, D.CARRIER_ROLL_CEIL - _HEAD_H/2) * Cylinder(_HEAD_D/2, _HEAD_H)
        _heads = _h if _heads is None else _heads + _h
    ok &= check("4X yaw-horn screw heads vs roll servo", vol(_heads, _roll_sv))
    _gap = D.CARRIER_ROLL_CEIL - _roll_sv.bounding_box().max.Z
    print(f"  {'servo-top to bay-ceiling gap':58s} {_gap:8.2f} mm   "
          f"{'OK' if _gap >= _HEAD_H else '** TOO TIGHT **'}")
    ok &= _gap >= _HEAD_H

    print("== yoke_pitch vs thigh (leg_link + servo), hip -110/+60 (+5 margin) ==")
    # flexion is NEGATIVE here (knee swings toward +x). The full leg_link rides
    # in this sweep: its idler grip plate / web share the idler arm's Y band,
    # and THEY (not the servo) set the mechanical limit.
    thigh_assy = ll + servo_mock_y()
    for ang in (0, -60, -95, -105, -110, -115, 60, 65):
        sv = Rot(0, ang, 0) * thigh_assy
        ok &= check(f"thigh assy at hip {ang:+d} deg", vol(yp, sv))
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
    ft = parts.foot()
    ankle_z = D.FOOT_T - D.FOOT_POCKET_D + 12.36        # 16.36 above sole bottom
    ankle_sv = Pos(0, 0, ankle_z) * Rot(0, 0, 0) * (
        # ankle servo: axis Y, length along X (output end forward +10.11), lying flat
        Rot(0, 90, 0) * servo_mock_y())  # rotate: Z(length) -> X
    for ang in (0, 40, -40):
        sl = Pos(0, 0, ankle_z) * Rot(0, ang, 0) * Pos(0, 0, D.LINK_DROP) * parts.leg_link()
        ok &= check(f"shin link at ankle {ang:+d} vs foot", vol(ft, sl))
        ok &= check(f"shin link at ankle {ang:+d} vs ankle servo", vol(sl, ankle_sv))
    ok &= check("ankle servo vs foot (contact only)", vol(ft, ankle_sv))

    print("== ankle-servo CABLE connector vs shin fork through ankle ROM ==")
    # The lead plugs into the ankle servo's cable-end (the HEEL end face, X=-35.11).
    # As the ankle rotates the foot+servo, that connector sweeps; the concern is it
    # fouling the shin fork's idler plate at toes-pointed (ankle +40, toe down).
    # Mock the connector+plug generously (full case width/height, protruding past
    # the heel face) and require 0 mm3 across the ROM -- there is ~10 mm clearance
    # (design review 2026-07-23; a speculative fork notch was removed after this).
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
