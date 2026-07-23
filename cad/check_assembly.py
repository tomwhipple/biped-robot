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
    axis at z=0, top end +35.11, bottom -10.11."""
    case = parts.box(D.SV_BOTFACE, D.SV_TOPFACE, -12.36, 12.36,
                     -D.SV_AXIS_FROM_OUT_END, D.SV_AXIS_FROM_REAR)
    case -= parts.cyl_x(D.SV_IDLER_RECESS_D / 2, D.SV_BOTFACE - 0.01,
                        D.SV_IDLER_FACE, 0, 0)
    horn = parts.cyl_x(D.SV_BOSS_D / 2, D.SV_TOPFACE, D.SV_HORN_FACE, 0, 0)
    return case + horn


def servo_mock_z():
    """Yaw servo (v3yaw): output axis VERTICAL, horn DOWN. Case length along X
    (YAW_CASE_X_REAR..FRONT), width along Y, thickness along Z. Origin at the
    output axis on the case mid-plane; idler-side (recessed O25) face UP, horn
    boss DOWN."""
    case = parts.box(D.YAW_CASE_X_REAR, D.YAW_CASE_X_FRONT, -12.36, 12.36,
                     D.SV_BOTFACE, D.SV_TOPFACE)
    case -= parts.cyl_z(D.SV_IDLER_RECESS_D / 2, 16.80, D.SV_TOPFACE + 0.01, 0, 0)
    horn = parts.cyl_z(D.SV_BOSS_D / 2, -D.SV_HORN_FACE, D.SV_BOTFACE, 0, 0)
    return case + horn


def vol(a, b):
    i = a & b
    return 0.0 if i is None else i.volume


def check(label, v, tol=1.0):
    print(f"  {label:58s} {v:8.2f} mm3  {'OK' if v <= tol else '** COLLISION **'}")
    return v <= tol


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

print("\nALL CLEAR" if ok else "\nINTERFERENCES FOUND — fix before printing")
