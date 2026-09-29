"""Hip-yaw bearing OPTION E: option A's 6810-2RS round hub, plus POSITIVE
retention of both races (docs/design-v6/study-yaw-bearing.md, 2026-09-18
round 3). Tom's review of option A: a press fit in printed PETG will creep
and the bearing will fall out. Option A has exactly ONE positive stop (the
pelvis shoulder above the outer race); the other three directions are
interference-fit friction only. This option adds the missing three:

  carrier (leg) side, inner race (band z -7..0, carrier-local, unchanged):
    - LIP  : printed ring under the race, z [-8.3, -6.8], r 25.04..27.5. The
             race now goes on from the horn-face side, down onto the lip.
    - CAP  : a separate thin ring ON TOP of the race, r 17..28.5 x 1.0 mm,
             in the existing 1.8 mm gap between the horn face and the cell
             rim, held by 3x M2.5 flat-head self-tappers at r 22 driven into
             the hub (7 mm of solid hub there, outside the bay cavity).
  pelvis side, outer race (recess z -80.8..-73.8, pelvis-local, unchanged):
    - shoulder above: UNCHANGED from option A.
    - BOSSES: 3 round bosses on the skirt OD (r 38.5, dia 8, hanging from
             the same cell rim down to the skirt bottom), M2.5 self-tap
             pilots from below.
    - RETAINER: a separate ring UNDER the skirt, 1.5 mm, land r 29.8..32.48
             on the outer race's bottom face, with 3 tabs to the bosses.

Load paths after this option (all positive, no friction relied on):
  stance (pelvis pushes DOWN on the leg): shoulder -> outer race -> balls ->
    inner race -> LIP -> carrier.
  swing (leg HANGS from the pelvis): carrier -> CAP screws -> CAP -> inner
    race -> balls -> outer race -> RETAINER -> boss screws -> pelvis.
The estimated ring split (V.YAWA_EST_*) sets the cap OD (must not touch the
outer race) and the retainer ID (must not touch the inner race) -- both
still ESTIMATES, same caveat as options A/C.

    .venv/bin/python cad/v6/yaw_retention_optE.py   # -> step/yaw_bearing_optE/
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))

from build123d import Pos, Rot, Compound, Box, Cone, export_step  # noqa: E402
import dimensions_v6 as V  # noqa: E402
import parts as P  # noqa: E402
import check_assembly as CA  # noqa: E402
import pelvis_v7  # noqa: E402
import yaw_carrier_v6_optA  # noqa: E402

D = V.D
box, cyl_z = P.box, P.cyl_z

# ---- carrier side (carrier-local, horn face z = 0)
E_HUB_R = V.YAWA_BRG_BOSS_R                 # 25.04, unchanged
E_LIP_R = 27.5                              # under the inner race (OD est 28.04): 0.54 mm short of the seal gap
# PRELOAD: the whole bearing (both races share their faces) sits E_PRELOAD
# higher than option A, so the race stands proud of the horn face by that
# much; the cap and the retainer each bottom on a RACE face while a
# E_PRELOAD gap remains to the plastic behind them (hub face / boss faces).
# Tightening the screws closes that gap onto the race, not the print, so
# print tolerance cannot leave the race loose -- this is what kills axial
# and tilt play. Radial play is a fit/retaining-compound matter, see doc.
E_PRELOAD = 0.2
E_RACE_Z = (V.YAWA_BAND_Z[0] + E_PRELOAD, V.YAWA_BAND_Z[1] + E_PRELOAD)   # (-6.8, 0.2) carrier-local
E_LIP_Z = (E_RACE_Z[0] - 1.5, E_RACE_Z[0])  # (-8.3, -6.8)
E_CAP_ID_R, E_CAP_OD_R = 17.0, 28.5         # clears the horn disc (15.97) / the outer race ID est 29.54 by 1.04
E_CAP_T = 1.0                               # cap z E_PRELOAD..E_PRELOAD+1.0; flat heads countersunk FLUSH, the
                                            # 90-deg cone continues 0.3 into the hub (solid there)
E_CAP_Z = (E_RACE_Z[1], E_RACE_Z[1] + E_CAP_T)   # (0.2, 1.2): 0.6 mm to the cell rim at 1.8
E_CAP_SCREW_R = 22.0                        # 3x M2.5 flat-head self-tap, into the solid hub (heads reach r 24.35 < hub 25.04)
E_CAP_SCREW_ANGLES = (90, 210, 330)         # +y first: all three land outside the bay cavity (|y|>12.66 or |x|>17.35)
E_PILOT = D.CASE_SCREW_PILOT                # 2.05, the build's M2.5 self-tap pilot
E_M25_CLEAR = 2.7
E_M25_HEAD_R, E_M25_HEAD_H = 2.35, 1.5      # flat/countersunk head
assert E_CAP_Z[1] + 0.4 <= (V.YAW_BOX_BOT_Z - V.YAW_HORN_FACE_Z), "cap (heads flush) must clear the cell rim by 0.4"
assert E_CAP_OD_R < V.YAWA_EST_OUTER_RING_ID, "cap must not touch the outer race"
assert E_LIP_R < V.YAWA_EST_OUTER_RING_ID, "lip must not reach the outer race"
for a in E_CAP_SCREW_ANGLES:
    import math
    x, y = E_CAP_SCREW_R * math.cos(math.radians(a)), E_CAP_SCREW_R * math.sin(math.radians(a))
    assert abs(x) > D.SV_TOPFACE + E_M25_CLEAR / 2 or abs(y) > D.SV_WID / 2 + D.BAY_CHEEK_GAP + E_M25_CLEAR / 2, \
        f"cap screw at {a} deg lands over the bay cavity"

# ---- pelvis side (pelvis-local, deck top z = 0)
E_BOSS_R_POS = 38.5                         # boss centre radius from the yaw axis
E_BOSS_R = 4.0                              # dia 8 boss
E_BOSS_Z = (V.YAWA_RECESS_Z[0] - 0.5, V.YAW_BOX_BOT_Z)   # (-81.3, -72.0): same span as the skirt
E_RET_T = 1.5
E_RET_Z = (E_BOSS_Z[0] - E_PRELOAD - E_RET_T, E_BOSS_Z[0] - E_PRELOAD)   # (-83.0, -81.5): body top E_PRELOAD below the skirt/boss faces
E_RET_ID_R = 29.8                           # land starts 0.26 outside the outer race ID est, 1.76 clear of the inner race
E_RET_LAND_OD_R = V.YAWA_BRG_RECESS_R - 0.2 # 32.28: the land enters the recess bore with 0.2 radial clearance
E_RET_OD_R = V.YAWA_SKIRT_OD / 2            # 35.5, flush with the skirt
E_RACE_BOT_PELVIS = V.YAWA_RECESS_Z[0] + E_PRELOAD   # -80.6 pelvis-local: outer race bottom face
E_RET_LAND_Z = (E_RET_Z[1], E_RACE_BOT_PELVIS)       # (-81.5, -80.6): 0.9 tall land up into the recess to the race
E_SHOULDER_Z = (V.YAWA_RECESS_Z[1] + E_PRELOAD, V.YAWA_SHOULDER_Z[1])   # (-73.6, -72.0): 1.6 mm land, was 1.8
assert E_SHOULDER_Z[1] - E_SHOULDER_Z[0] >= 1.2, "pelvis shoulder land too thin"
E_RET_SCREW_DEPTH = 7.0                     # M2.5x8 self-tap: 1.5 ring + 6.5 in the boss
assert E_RET_ID_R > V.YAWA_EST_INNER_RING_OD + 1.5, "retainer must clear the inner race"
assert E_RET_ID_R > E_LIP_R + 2.0, "retainer must clear the carrier lip"
assert E_BOSS_R_POS + E_BOSS_R < V.HIP_SEP - (V.YAWA_SKIRT_OD / 2) - 4.0, "bosses must stay clear of the other hip's skirt"


def boss_angles(y_hip):
    """Outboard first, then +-120 deg: keeps all three bosses off the
    inboard line where the skirt bridge is."""
    out = 90 if y_hip > 0 else 270
    return (out, out + 120, out + 240)


def _ring(r0, r1, z0, z1, x=0, y=0):
    return cyl_z(r1, z0, z1, x, y) - cyl_z(r0, z0 - 1, z1 + 1, x, y)


def _pol(r, ang):
    import math
    return r * math.cos(math.radians(ang)), r * math.sin(math.radians(ang))


def carrier_optE(print_fins=False):
    """Option A's carrier + the printed lip under the inner race + 3 pilots
    for the cap screws."""
    p = yaw_carrier_v6_optA.yaw_carrier_v6_optA(print_fins=print_fins)
    p += _ring(E_HUB_R - 0.5, E_LIP_R, *E_LIP_Z)     # overlaps the hub by 0.5 so it fuses
    for a in E_CAP_SCREW_ANGLES:
        x, y = _pol(E_CAP_SCREW_R, a)
        p -= cyl_z(E_PILOT / 2, -6.0, 0.5, x, y)
        p -= _head_cone(x, y)                        # the countersink's tail, below the cap
    return p


def _head_cone(x, y):
    """90-deg countersink for an M2.5 flat head, top flush with the cap's top
    face; the same cone is cut from the cap AND the hub beneath it."""
    top = E_CAP_Z[1]
    return Pos(x, y, top - E_M25_HEAD_H / 2 + 0.005) * Cone(0.0, E_M25_HEAD_R, E_M25_HEAD_H + 0.01)


def cap():
    """Carrier-local. Thin ring over the inner race, countersunk for 3x M2.5 flat heads."""
    c = _ring(E_CAP_ID_R, E_CAP_OD_R, *E_CAP_Z)
    for a in E_CAP_SCREW_ANGLES:
        x, y = _pol(E_CAP_SCREW_R, a)
        c -= cyl_z(E_M25_CLEAR / 2, E_CAP_Z[0] - 1, E_CAP_Z[1] + 1, x, y)
        c -= _head_cone(x, y)
    return c


def pelvis_optE(arm_mounts=True):
    """Option A's pelvis + 3 screw bosses on each hip skirt. arm_mounts: the
    shoulder girdle's deck pilots (the robot's build; False = ARMS=0)."""
    p = pelvis_v7.pelvis_v7(bearing_variant="A", arm_mounts=arm_mounts)
    for y_hip in (V.HIP_SEP / 2, -V.HIP_SEP / 2):
        # raise the recess top (= the shoulder's underside) by E_PRELOAD so the
        # race stands proud of the horn face on the carrier side
        p -= cyl_z(V.YAWA_BRG_RECESS_R, V.YAWA_RECESS_Z[1] - 0.01, E_SHOULDER_Z[0], 0, y_hip)
        for a in boss_angles(y_hip):
            x, y = _pol(E_BOSS_R_POS, a)
            # web from the skirt wall out to the boss, then the boss itself
            p += cyl_z(E_BOSS_R, *E_BOSS_Z, x, y + y_hip)
            xw, yw = _pol((E_BOSS_R_POS + E_RET_OD_R) / 2, a)
            p += Pos(xw, yw + y_hip, sum(E_BOSS_Z) / 2) * Rot(0, 0, a) * \
                Box(E_BOSS_R_POS - E_RET_OD_R + 2, 2 * E_BOSS_R, E_BOSS_Z[1] - E_BOSS_Z[0])
            p -= cyl_z(E_PILOT / 2, E_BOSS_Z[0] - 1, E_BOSS_Z[0] + E_RET_SCREW_DEPTH - E_RET_T, x, y + y_hip)
    return p


def retainer(y_hip):
    """Pelvis-local, one hip. Ring under the skirt bearing on the outer race's
    bottom face, with 3 tabs out to the bosses."""
    r = _ring(E_RET_ID_R, E_RET_OD_R, *E_RET_Z, 0, y_hip)
    r += _ring(E_RET_ID_R, E_RET_LAND_OD_R, E_RET_LAND_Z[0] - 0.01, E_RET_LAND_Z[1], 0, y_hip)   # the land, up to the race
    for a in boss_angles(y_hip):
        x, y = _pol(E_BOSS_R_POS, a)
        r += cyl_z(E_BOSS_R, *E_RET_Z, x, y + y_hip)
        xw, yw = _pol((E_BOSS_R_POS + E_RET_OD_R) / 2, a)
        r += Pos(xw, yw + y_hip, sum(E_RET_Z) / 2) * Rot(0, 0, a) * \
            Box(E_BOSS_R_POS - E_RET_OD_R + 2, 2 * E_BOSS_R, E_RET_T)
        r -= cyl_z(E_M25_CLEAR / 2, E_RET_Z[0] - 1, E_RET_Z[1] + 1, x, y + y_hip)
    return r


def SCREWS():
    """Option E's screws for ONE hip. Cap: 3 M2.5 flat-heads, heads flush in
    the cap's top face (carrier-local z E_CAP_Z[1]), into 2.05 pilots that
    run to z -6.0 in the hub -- an M2.5 x 6 ends 1.2 mm short of the pilot
    bottom (an x 8 would bottom out). Retainer: 3 M2.5 x 8 flat-heads UP
    through the ring into the skirt bosses (pelvis-local, +y hip shown).
    `axis` points from the head toward the tip."""
    s = []
    for a in E_CAP_SCREW_ANGLES:
        x, y = _pol(E_CAP_SCREW_R, a)
        s.append(dict(name=f"cap_{a}", frame="carrier", kind="M2.5x6 self-tap, flat head (cap into the hub)",
                      pos=(x, y, E_CAP_Z[1]), axis=(0, 0, -1), length=6.0))
    for a in boss_angles(V.HIP_SEP / 2):
        x, y = _pol(E_BOSS_R_POS, a)
        s.append(dict(name=f"retainer_{a}", frame="pelvis",
                      kind="M2.5x8 self-tap, flat head (retainer up into the skirt boss)",
                      pos=(x, y + V.HIP_SEP / 2, E_RET_Z[0]), axis=(0, 0, 1), length=8.0))
    assert E_CAP_Z[1] - 6.0 > -6.0, "the cap screw must end above its pilot's bottom"
    return s


def build_joint():
    """World-frame joint sub-assembly for the +y hip, same convention as
    export_yaw_bearing_joint.py: cropped pelvis cell + carrier + races +
    servo mocks + the two new retention parts."""
    y = V.HIP_SEP / 2
    carrier = carrier_optE()
    pelvis = pelvis_optE()
    ring_z = (V.HIP_YAW_Z + E_RACE_Z[0], V.HIP_YAW_Z + E_RACE_Z[1])
    inner = Pos(0, y, 0) * _ring(E_HUB_R, V.YAWA_EST_INNER_RING_OD, *ring_z)
    outer = Pos(0, y, 0) * _ring(V.YAWA_EST_OUTER_RING_ID, V.YAWA_BRG_RECESS_R, *ring_z)
    carrier_w = Pos(0, y, V.HIP_YAW_Z) * carrier
    cap_w = Pos(0, y, V.HIP_YAW_Z) * cap()
    ret_w = Pos(0, 0, V.DECK_TOP_Z) * retainer(y)
    yaw_servo_w = Pos(0, y, V.HIP_YAW_Z + D.SV_HORN_FACE) * CA.servo_mock_z()
    roll_servo_w = Pos(0, y, V.HIP_ROLL_Z) * CA.servo_mock_x()
    pelvis_w = Pos(0, 0, V.DECK_TOP_Z) * pelvis
    crop = Pos(-15, y, V.YAW_BOX_BOT_Z / 2 + V.DECK_TOP_Z) * Box(90, 90, -V.YAW_BOX_BOT_Z + 20)
    pelvis_cell = pelvis_w & crop

    named = [(pelvis_cell, "pelvis_v7_cell_E"), (carrier_w, "yaw_carrier_E"),
             (inner, "bearing_inner_race"), (outer, "bearing_outer_race"),
             (cap_w, "inner_race_cap"), (ret_w, "outer_race_retainer"),
             (yaw_servo_w, "servo_hip_yaw_mock"), (roll_servo_w, "servo_hip_roll_mock")]
    for solid, label in named:
        solid.label = label
    joint = Compound(label="yaw_joint_E", children=[s for s, _ in named])
    return carrier, pelvis, cap(), retainer(y), joint


def clearance_report(joint):
    """Pairwise intersection volumes between the parts that must not touch."""
    parts = {c.label: c for c in joint.children}
    must_clear = [("inner_race_cap", "bearing_outer_race"), ("inner_race_cap", "pelvis_v7_cell_E"),
                  ("inner_race_cap", "servo_hip_yaw_mock"), ("outer_race_retainer", "bearing_inner_race"),
                  ("outer_race_retainer", "yaw_carrier_E"), ("outer_race_retainer", "servo_hip_roll_mock"),
                  ("yaw_carrier_E", "pelvis_v7_cell_E"), ("yaw_carrier_E", "bearing_outer_race"),
                  ("pelvis_v7_cell_E", "bearing_inner_race"),
                  ("inner_race_cap", "yaw_carrier_E"), ("outer_race_retainer", "pelvis_v7_cell_E"),
                  ("inner_race_cap", "bearing_inner_race"), ("outer_race_retainer", "bearing_outer_race"),
                  ("yaw_carrier_E", "bearing_inner_race"), ("pelvis_v7_cell_E", "bearing_outer_race")]
    out = []
    for a, b in must_clear:
        vol = (parts[a] & parts[b]).volume
        out.append((a, b, vol))
    return out


if __name__ == "__main__":
    folder = os.path.join(HERE, "step", "yaw_bearing_optE")
    os.makedirs(folder, exist_ok=True)
    carrier, pelvis, cap_s, ret_s, joint = build_joint()
    for name, s in (("yaw_carrier_v6_E", carrier), ("pelvis_v7_E", pelvis),
                    ("inner_race_cap", cap_s), ("outer_race_retainer", ret_s), ("joint_assembly_E", joint)):
        export_step(s, os.path.join(folder, f"{name}.step"))
    print(f"carrier {carrier.volume/1000:.2f} cm3, pelvis {pelvis.volume/1000:.2f} cm3, "
          f"cap {cap_s.volume/1000:.3f} cm3, retainer {ret_s.volume/1000:.3f} cm3")
    for a, b, vol in clearance_report(joint):
        print(f"  {a:22s} x {b:22s} overlap {vol:8.3f} mm3 {'OK' if vol < 1e-3 else 'COLLISION'}")
    print(f"wrote {folder}/")
