"""Parametric MJCF generator for the v6 body: 6 DOF per leg (hip yaw / roll /
pitch, knee, ankle pitch, ankle ROLL), 12 STS3215-class servos.

Why this exists (docs/lessons-learned-2026-09-13-walking.md, docs/design-stage-
simulation-gates.md): the 10-DOF body has no ankle roll, so a planted foot
pins the pelvis level and the centre of mass cannot be put over one foot from
rest. The v6 design adds the ankle roll and is sized here BEFORE any CAD, so
the Gate A-D questions ("can this body stand on one foot with margin, with
this servo, at this cadence?") are answered on the kinematics and the servo
envelope, not on a policy.

Everything is a parameter (DesignParams). Masses are ESTIMATES until CAD
exists: servos at their catalogue mass in their real case envelope at the
real place in the chain (this is 55-60 % of the robot and dominates the
inertia), printed parts as lumped boxes at the masses the current parts
weigh (cad/PRINT_LIST.md), battery/board at catalogue mass. When the CAD
lands, sim/build_v2_inertia.py replaces the lumps -- same as v5body.

Conventions kept from sim/bimo_biped_v5body.xml so walker_env / the tools
work unchanged: +X forward, +Y left, torso freejoint, joint names
{L,R}_{hip_yaw,hip_roll,hip_pitch,knee,ankle,ankle_roll}, sim-negative knee =
human flexion (axis 0 -1 0), 8 pad spheres per sole as the only ground
contact, non-colliding L_sole/R_sole reference boxes, enumerated inter-leg
contact pairs, IMU site + torso_up/torso_pos sensors.

    .venv/bin/python sim/gen_plant_v6.py                 # -> sim/bimo_biped_v6ar.xml
    .venv/bin/python sim/gen_plant_v6.py --knee bwd -o /tmp/bwd.xml
    .venv/bin/python sim/gen_plant_v6.py --set hip_sep=0.070 thigh=0.11 -o /tmp/x.xml
"""
from __future__ import annotations

import argparse
import dataclasses
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_OUT = os.path.join(HERE, "bimo_biped_v6ar.xml")

# STS3215 / STS3250 case (cad/dimensions.py, STEP-measured): 45.22 long x
# 24.72 wide x 34.70 across the output axis; output axis 10.11 from the
# output end, centred in the width.
SV_LEN, SV_WID, SV_T = 0.04522, 0.02472, 0.03470
SV_AX_OUT = 0.01011


@dataclasses.dataclass
class DesignParams:
    # ---- kinematics (m) ---------------------------------------------------
    hip_sep: float = 0.084       # leg centre-to-centre (v5: 0.066). Narrower = less roll torque
                                 # (sweep), but the swing foot drifts ~13 mm inward under roll
                                 # sag/play and landed ON the stance foot at a 12 mm gap; 84 gives
                                 # a 24 mm gap and a 32 mm battery channel between the yaw servos
    d_yaw_roll: float = 0.041    # hip yaw axis -> hip roll axis (v5 stack)
    d_roll_pitch: float = 0.050  # hip roll axis -> hip pitch axis (yoke pair)
    thigh: float = 0.110         # hip pitch -> knee (v5: 0.090; longer = lower joint rates for the same step)
    shank: float = 0.110         # knee -> ankle pitch (v5: 0.090)
    d_ankle: float = 0.058       # ankle pitch axis -> ankle roll axis (the pitch servo HANGS
                                 # below its axis like every other joint; 50 = the hip's
                                 # ROLL_TO_PITCH, so the ankle link is a short leg_link)
    roll_h: float = 0.01836      # ankle roll axis above the sole bottom: 4 mm plate + 12.36
                                 # (axis centred in the 24.72 case width) + 2 mm TPU sole
    foot_len: float = 0.130
    foot_w: float = 0.084        # sole width; with foot_y_off the inboard half is 30 mm
                                 # (inner gap 24 mm), the outboard half 54 mm (2026-09-14)
    foot_toe: float = 0.075      # ankle axis -> toe edge (heel = len - toe)
    foot_r: float = 0.014        # corner radius (pad octagon like v5)
    foot_y_off: float = 0.012    # sole centreline OUTBOARD of the ankle roll axis (m):
                                 # an asymmetric sole with more width outside the ankle,
                                 # because the open-loop walk's failure direction is outward
    knee: str = "fwd"            # "fwd" = human knee, "bwd" = bird knee
    # ---- joint ranges (deg) ----------------------------------------------
    yaw_range: float = 45.0
    hip_roll_add: float = 30.0   # adduction (toward the other leg)
    hip_roll_abd: float = 45.0   # abduction
    hip_pitch_range: tuple = (-110.0, 90.0)
    knee_flex: float = 95.0      # flexion travel (either direction)
    knee_hyper: float = 5.0      # hyperextension cap (modelling cap, v5)
    ankle_range: float = 45.0    # ankle pitch +/-
    ankle_roll_range: float = 25.0   # the roll servo case sweep under the ankle link (cad/v6/dimensions_v6.py)
    # ---- masses (kg) ------------------------------------------------------
    servo_mass: float = 0.055    # STS3215; STS3250 = 0.0745
    m_pelvis: float = 0.174      # v7 torso print (CAD 2026-09-14 final: 174.2 g)
    m_carrier: float = 0.0175    # yaw carrier (PRINT_LIST)
    m_hip_yokes: float = 0.030   # yoke_roll + yoke_pitch (v5 L_hip 29.6 g)
    m_leg_link: float = 0.027    # leg_link_v6 (CAD 2026-09-14: 26.7 g)
    m_ankle_link: float = 0.009  # ankle_link (CAD 2026-09-14: 8.9 g)
    m_foot: float = 0.071        # foot_v6 47 g + TPU sole 23 g (CAD 2026-09-14)
    m_battery: float = 0.170     # 3S 2200-2600 mAh class LiPo (150-190 g)
    m_board: float = 0.030       # General Driver 65x65
    m_wiring: float = 0.030      # leads, ties, switch
    payload_ref: float = 0.0     # Pi + camera volume reserved; mass via DR
    arms: bool = False           # get-up study: two 1-DOF stub arms (shoulder pitch, STS3215) on
                                 # the deck sides, rigid links with a rubber tip
    arm_len: float = 0.20
    arm_mass: float = 0.020      # printed link + tip
    arm_shoulder_x: float = -0.02
    fall_collision: bool = True  # torso, head, links, feet collide with the FLOOR (contype 2),
                                 # so falls and get-ups are physical; soles stay the 8 pads
    ankle_servos_in_shank: bool = False   # study variant: parallel-linkage ankle --
                                          # both ankle servos ride at the top of the
                                          # shank, the ankle joints are driven through
                                          # links (modelled: same joints, servo mass
                                          # moved up, linkage lumps at the ankle)
    m_linkage: float = 0.015     # per ankle DOF: rod ends + link + lever
    # ---- torso geometry (m) ----------------------------------------------
    deck_x: tuple = (-0.058, 0.052)
    deck_w: float = 0.118
    deck_t: float = 0.005
    housing_h: float = 0.043     # yaw axis -> deck bottom (v5: 0.0428)
    # ---- torso v7 (2026-09-13, Tom: full Pi 4B, taller torso, head on a
    # yaw servo). torso_v7=True raises the deck by a transverse battery layer
    # above the yaw cells, stands the Pi 4B on the aft wall and the General
    # Driver on the front wall, and adds a neck STS3215 + head above the deck.
    torso_v7: bool = True
    batt_layer_h: float = 0.031  # battery layer between the yaw cells and the deck
    m_pi4: float = 0.066         # Pi 4B 46 g + heatsink/standoffs 20 g
    m_power: float = 0.060       # 3S protection/UPS module + 5 V buck + leads
    m_neck_servo: float = 0.055  # STS3215
    m_head: float = 0.039        # head 34.7 g + Camera Module 3 (CAD 2026-09-14)
    head_h: float = 0.050        # neck horn face -> head CoM

    @property
    def z_hip_pitch_above_sole(self) -> float:
        return self.roll_h + self.d_ankle + self.shank + self.thigh

    @property
    def z_yaw_above_sole(self) -> float:
        return self.z_hip_pitch_above_sole + self.d_roll_pitch + self.d_yaw_roll

    @property
    def deck_bot(self) -> float:
        """deck underside above the torso origin (yaw axis)."""
        return self.housing_h + (self.batt_layer_h if self.torso_v7 else 0.0)

    @property
    def knee_range(self) -> tuple[float, float]:
        # axis is (0 -1 0): negative q = human flexion
        if self.knee == "fwd":
            return (-self.knee_flex, self.knee_hyper)
        if self.knee == "bwd":
            return (-self.knee_hyper, self.knee_flex)
        raise ValueError(self.knee)

    def summary(self) -> str:
        return (f"hip_sep {1e3*self.hip_sep:.0f}  thigh {1e3*self.thigh:.0f}  "
                f"shank {1e3*self.shank:.0f}  d_ankle {1e3*self.d_ankle:.0f}  "
                f"foot {1e3*self.foot_len:.0f}x{1e3*self.foot_w:.0f}  knee {self.knee}  "
                f"{'torso v7 (Pi 4B + head)  ' if self.torso_v7 else ''}"
                f"yaw axis {1e3*self.z_yaw_above_sole:.0f} mm, deck top "
                f"{1e3*(self.z_yaw_above_sole+self.deck_bot+self.deck_t):.0f} mm")


def _fc(p) -> str:
    return 'class="fallcol" ' if p.fall_collision else ''


def _f(x: float) -> str:
    return f"{x:.5f}".rstrip("0").rstrip(".") if abs(x) > 1e-9 else "0"


def _pads(p: DesignParams, side: str) -> str:
    """8 pad spheres on the inscribed octagon of the rounded-rect sole, sole
    bottom at foot-frame z = -roll_h (== v5 convention: r 3 mm, centre 3 mm up).
    The sole centreline sits foot_y_off OUTBOARD of the ankle roll axis."""
    r = 0.003
    zc = -p.roll_h + r
    x_heel = -(p.foot_len - p.foot_toe)
    x_toe = p.foot_toe
    hw = p.foot_w / 2
    cr = p.foot_r
    yo = p.foot_y_off if side == "L" else -p.foot_y_off
    pts = [
        ("hl", x_heel, +hw - cr), ("hr", x_heel, -(hw - cr)),
        ("hol", x_heel + cr, +hw), ("hor", x_heel + cr, -hw),
        ("tol", x_toe - cr, +hw), ("tor", x_toe - cr, -hw),
        ("tl", x_toe, +hw - cr), ("tr", x_toe, -(hw - cr)),
    ]
    return "\n".join(
        f'              <geom name="{side}_pad_{n}" class="pad" pos="{_f(x)} {_f(y + yo)} {_f(zc)}"/>'
        for n, x, y in pts)


def _shank_ankle_servos(p: DesignParams, side: str) -> str:
    """parallel-ankle study variant: both ankle servos stacked in the shank just
    below the knee servo (cases side by side across the shank), driving the
    ankle through links (not modelled as geometry)."""
    if not p.ankle_servos_in_shank:
        return ""
    sv = p.servo_mass
    z1 = -(SV_LEN - SV_AX_OUT) - 0.004 - SV_WID / 2      # below the knee servo case
    return (f'              <geom class="servo" type="box" pos="0.004 0.009 {_f(z1)}" size="{_f(SV_WID/2)} {_f(SV_T/4)} {_f(SV_LEN/2*0.55)}" mass="{sv}"/>\n'
            f'              <geom class="servo" type="box" pos="0.004 -0.009 {_f(z1)}" size="{_f(SV_WID/2)} {_f(SV_T/4)} {_f(SV_LEN/2*0.55)}" mass="{sv}"/>')


def _leg(p: DesignParams, side: str) -> str:
    s = +1.0 if side == "L" else -1.0
    y = s * p.hip_sep / 2
    # hip roll: + rotates the toe toward robot LEFT (v5 convention), so on the
    # LEFT leg + is abduction, on the RIGHT leg + is adduction.
    if side == "L":
        roll_rng = (-p.hip_roll_add, p.hip_roll_abd)
    else:
        roll_rng = (-p.hip_roll_abd, p.hip_roll_add)
    ar = p.ankle_roll_range
    kr = p.knee_range
    hp = p.hip_pitch_range
    sv = p.servo_mass
    servo_rgba = "0.22 0.23 0.27 1"
    link_rgba = "0.82 0.84 0.87 1"
    # servo half-sizes by output-axis direction
    # axis Y (pitch joints): thickness along Y, length along Z (axis 10.1 from
    # the output end), width along X
    sy = (SV_WID / 2, SV_T / 2, SV_LEN / 2)
    # axis X (roll joints in the hip carrier): thickness along X, length along Z
    sx_v = (SV_T / 2, SV_WID / 2, SV_LEN / 2)
    # axis X, lying ACROSS the foot: thickness along X, length along Y, width along Z
    sx_h = (SV_T / 2, SV_LEN / 2, SV_WID / 2)
    # axis Z (yaw): thickness along Z, length along X
    z_servo_case_dn = -(SV_LEN / 2 - SV_AX_OUT)   # case centre when the case hangs below the axis
    z_servo_case_up = +(SV_LEN / 2 - SV_AX_OUT)   # case centre when the case rises above the axis
    foot_cx = p.foot_toe - p.foot_len / 2
    foot_cy = p.foot_y_off if side == "L" else -p.foot_y_off
    sole_z = -p.roll_h
    return f"""
      <body name="{side}_hip_yaw" pos="0 {_f(y)} 0">
        <joint name="{side}_hip_yaw" axis="0 0 1" range="{-p.yaw_range:.0f} {p.yaw_range:.0f}"/>
        <!-- yaw carrier print + the hip ROLL servo riding in it (axis X) -->
        <geom type="box" pos="0 0 {_f(-p.d_yaw_roll/2)}" size="0.014 0.020 {_f(p.d_yaw_roll/2)}" mass="{p.m_carrier}" rgba="{link_rgba}" group="1"/>
        <geom class="servo" type="box" pos="0 0 {_f(-p.d_yaw_roll + z_servo_case_up)}" size="{_f(sx_v[0])} {_f(sx_v[1])} {_f(sx_v[2])}" mass="{sv}"/>
        <body name="{side}_hip" pos="0 0 {_f(-p.d_yaw_roll)}">
          <joint name="{side}_hip_roll" axis="1 0 0" range="{roll_rng[0]:.0f} {roll_rng[1]:.0f}"/>
          <!-- yoke_roll + yoke_pitch (the hip universal, 50 mm axis offset) -->
          <geom type="box" pos="0 0 {_f(-p.d_roll_pitch/2)}" size="0.0175 0.0125 {_f(p.d_roll_pitch/2)}" mass="{p.m_hip_yokes}" rgba="{link_rgba}" group="1"/>
          <body name="{side}_thigh" pos="0 0 {_f(-p.d_roll_pitch)}">
            <joint name="{side}_hip_pitch" axis="0 1 0" range="{hp[0]:.0f} {hp[1]:.0f}"/>
            <!-- hip-pitch servo case sits in the thigh, axis at its upper end -->
            <geom class="servo" type="box" pos="0 0 {_f(z_servo_case_dn)}" size="{_f(sy[0])} {_f(sy[1])} {_f(sy[2])}" mass="{sv}"/>
            <geom {_fc(p)}type="box" pos="0 0 {_f(-p.thigh/2)}" size="0.012 0.019 {_f(p.thigh/2)}" mass="{p.m_leg_link}" rgba="{link_rgba}" group="1"/>
            <geom name="{side}_col_thigh" class="legcol" fromto="0 0 0 0 0 -0.030"/>
            <body name="{side}_shin" pos="0 0 {_f(-p.thigh)}">
              <joint name="{side}_knee" axis="0 -1 0" range="{kr[0]:.0f} {kr[1]:.0f}"/>
              <geom class="servo" type="box" pos="0 0 {_f(z_servo_case_dn)}" size="{_f(sy[0])} {_f(sy[1])} {_f(sy[2])}" mass="{sv}"/>
              <geom {_fc(p)}type="box" pos="0 0 {_f(-p.shank/2)}" size="0.012 0.019 {_f(p.shank/2)}" mass="{p.m_leg_link}" rgba="{link_rgba}" group="1"/>
              <geom name="{side}_col_shank" class="legcol" fromto="0 0 0 0 0 -0.030"/>
{_shank_ankle_servos(p, side)}
              <body name="{side}_ankle_blk" pos="0 0 {_f(-p.shank)}">
                <joint name="{side}_ankle" axis="0 1 0" range="{-p.ankle_range:.0f} {p.ankle_range:.0f}"/>
                <!-- ankle-pitch servo case hangs below the axis inside the
                     ankle link (the hip's roll->pitch stack, upside down) -->
                {"" if p.ankle_servos_in_shank else f'<geom class="servo" type="box" pos="0 0 {_f(z_servo_case_dn)}" size="{_f(sy[0])} {_f(sy[1])} {_f(sy[2])}" mass="{sv}"/>'}
                <geom {_fc(p)}type="box" pos="0 0 {_f(-p.d_ankle/2)}" size="0.020 0.016 {_f(p.d_ankle/2)}" mass="{p.m_ankle_link if not p.ankle_servos_in_shank else p.m_ankle_link * 0.6 + p.m_linkage}" rgba="{link_rgba}" group="1"/>
                <geom name="{side}_col_ankle" type="box" pos="0 0 {_f(-p.d_ankle/2)}" size="0.021 0.017 {_f(p.d_ankle/2)}" contype="0" conaffinity="0" mass="0" group="4" rgba="0.85 0.35 0.25 0.3"/>
                <body name="{side}_foot" pos="0 0 {_f(-p.d_ankle)}">
                  <joint name="{side}_ankle_roll" axis="1 0 0" range="{-ar:.0f} {ar:.0f}"/>
                  <!-- ankle-ROLL servo lies across the foot, output axis X,
                       case bottom on the sole plate; the ankle link forks
                       onto its horn (front) and idler (rear) -->
                  {"" if p.ankle_servos_in_shank else f'<geom class="servo" type="box" pos="0 0 0" size="{_f(sx_h[0])} {_f(sx_h[1])} {_f(sx_h[2])}" mass="{sv}"/>'}
                  {f'<geom type="box" pos="0 0 0" size="0.010 0.015 0.008" mass="{p.m_linkage}" rgba="{link_rgba}" group="1"/>' if p.ankle_servos_in_shank else ""}
                  <geom {_fc(p)}type="box" pos="{_f(foot_cx)} {_f(foot_cy)} {_f(sole_z + 0.005)}" size="{_f(p.foot_len/2)} {_f(p.foot_w/2)} 0.003" mass="{p.m_foot}" rgba="0.30 0.31 0.34 1" group="1"/>
{_pads(p, side)}
                  <!-- reference sole (non-colliding; walker_env reads it) and
                       the full-footprint inter-foot collision proxy -->
                  <geom name="{side}_sole" type="box" pos="{_f(foot_cx)} {_f(foot_cy)} {_f(sole_z + 0.004)}" size="{_f(p.foot_len/2)} {_f(p.foot_w/2)} 0.004"
                        contype="0" conaffinity="0" group="3" mass="0" friction="1 0.02 0.001" condim="4"/>
                  <geom name="{side}_col_foot" type="box" pos="{_f(foot_cx)} {_f(foot_cy)} {_f(sole_z + 0.010)}" size="{_f(p.foot_len/2)} {_f(p.foot_w/2)} 0.010"
                        contype="0" conaffinity="0" group="3" mass="0" friction="1 0.02 0.001" condim="4"/>
                </body>
              </body>
            </body>
          </body>
        </body>
      </body>"""


def _torso(p: DesignParams) -> str:
    d = p.deck_x
    deck_cx = (d[0] + d[1]) / 2
    deck_hx = (d[1] - d[0]) / 2
    zdeck = p.deck_bot + p.deck_t / 2
    yaw_case = (SV_LEN / 2, SV_WID / 2, SV_T / 2)
    yaw_z = p.housing_h - SV_T / 2 - 0.003
    s = []
    s.append(f'      <!-- one-print pelvis: deck + housing walls -->')
    s.append(f'      <geom {_fc(p)}type="box" pos="{_f(deck_cx)} 0 {_f(zdeck)}" size="{_f(deck_hx)} {_f(p.deck_w/2)} {_f(p.deck_t/2)}" mass="{p.m_pelvis*0.5}" rgba="0.82 0.84 0.87 1" group="1"/>')
    s.append(f'      <geom {_fc(p)}type="box" pos="{_f(deck_cx)} 0 {_f(p.deck_bot/2)}" size="{_f(deck_hx*0.9)} {_f(p.deck_w/2 - 0.004)} {_f(p.deck_bot/2)}" mass="{p.m_pelvis*0.5}" rgba="0.82 0.84 0.87 0.25" group="1"/>')
    s.append(f'      <!-- hip YAW servos hang under the housing floor, horn down on the yaw axis -->')
    for sgn in (1, -1):
        s.append(f'      <geom class="servo" pos="{_f(-(SV_LEN/2 - SV_AX_OUT))} {_f(sgn*p.hip_sep/2)} {_f(yaw_z)}" size="{_f(yaw_case[0])} {_f(yaw_case[1])} {_f(yaw_case[2])}" mass="{p.servo_mass}"/>')
    if not p.torso_v7:
        s.append(f'      <geom type="box" pos="-0.010 0 0.018" size="0.031 0.016 0.013" mass="{p.m_battery}" rgba="0.15 0.35 0.75 1" group="1"/>')
        s.append(f'      <geom type="box" pos="{_f(d[0]+0.008)} 0 0.024" size="0.006 0.0325 0.020" mass="{p.m_board}" rgba="0.1 0.5 0.2 1" group="1"/>')
        s.append(f'      <geom type="box" pos="0.02 0 0.030" size="0.015 0.03 0.008" mass="{p.m_wiring}" rgba="0.3 0.3 0.3 0.4" group="1"/>')
        s.append(f'      <site name="imu" pos="{_f(d[0]+0.014)} 0 0.024" size="0.004" rgba="1 0 0 0.6"/>')
        s.append(f'      <body name="pi_bay" pos="0.028 0 {_f(p.housing_h + p.deck_t + 0.008)}">')
        s.append(f'        <geom type="box" size="0.033 0.016 0.008" mass="{p.payload_ref}" rgba="0.8 0.2 0.6 0.5" group="1"/>')
        s.append(f'      </body>')
        return "\n".join(s)
    # ---- v7: battery TRANSVERSE above the yaw cells, Pi 4B vertical on the
    # aft wall (85 across, 56 tall), General Driver vertical on the front wall,
    # power module under the deck beside the battery, neck servo + head on top
    zb = p.housing_h + p.batt_layer_h / 2
    s.append(f'      <!-- 3S 2200 mAh class pack, transverse, 105 x 36 x 26 -->')
    s.append(f'      <geom type="box" pos="-0.012 0 {_f(zb)}" size="0.018 0.0525 0.013" mass="{p.m_battery}" rgba="0.15 0.35 0.75 1" group="1"/>')
    s.append(f'      <!-- Pi 4B on the aft wall: 85 across (y), 56 tall (z), 20 deep with heatsink -->')
    s.append(f'      <geom type="box" pos="{_f(d[0]+0.011)} 0 {_f(p.deck_bot/2 + 0.004)}" size="0.010 0.0425 0.028" mass="{p.m_pi4}" rgba="0.1 0.5 0.2 1" group="1"/>')
    s.append(f'      <!-- General Driver on the front wall: 65 x 65, 12 deep -->')
    s.append(f'      <geom type="box" pos="{_f(d[1]-0.009)} 0 {_f(p.deck_bot/2 + 0.004)}" size="0.007 0.0325 0.0325" mass="{p.m_board}" rgba="0.1 0.5 0.2 1" group="1"/>')
    s.append(f'      <!-- power: 3S protection / UPS module + 5 V buck, beside the pack -->')
    s.append(f'      <geom type="box" pos="0.024 0 {_f(zb)}" size="0.014 0.030 0.010" mass="{p.m_power}" rgba="0.6 0.3 0.1 1" group="1"/>')
    s.append(f'      <geom type="box" pos="0.0 0 {_f(p.deck_bot - 0.010)}" size="0.02 0.03 0.006" mass="{p.m_wiring}" rgba="0.3 0.3 0.3 0.4" group="1"/>')
    s.append(f'      <site name="imu" pos="{_f(d[1]-0.016)} 0 {_f(p.deck_bot/2 + 0.004)}" size="0.004" rgba="1 0 0 0.6"/>')
    # neck servo: axis Z, case above the deck, horn up (the head body itself is
    # emitted by _head() AFTER the legs, so the neck joint is LAST in qpos)
    zn = p.deck_bot + p.deck_t
    s.append(f'      <!-- neck STS3215, axis Z, horn UP; the head yaws on it -->')
    s.append(f'      <geom class="servo" pos="{_f(-(SV_LEN/2 - SV_AX_OUT))} 0 {_f(zn + SV_T/2)}" size="{_f(yaw_case[0])} {_f(yaw_case[1])} {_f(yaw_case[2])}" mass="{p.m_neck_servo}"/>')
    s.append(f'      <body name="pi_bay" pos="{_f(d[0]+0.011)} 0 {_f(p.deck_bot/2 + 0.004)}">')
    s.append(f'        <geom type="box" size="0.002 0.002 0.002" mass="{p.payload_ref}" rgba="0.8 0.2 0.6 0.0" group="3"/>')
    s.append(f'      </body>')
    return "\n".join(s)


def _arms(p: DesignParams) -> str:
    if not p.arms:
        return ""
    out = []
    zs = p.deck_bot - 0.015
    for side, sgn in (("L", 1), ("R", -1)):
        y = sgn * (p.deck_w / 2 + SV_T / 2 + 0.004)
        out.append(f"""
      <body name="{side}_arm" pos="{_f(p.arm_shoulder_x)} {_f(y)} {_f(zs)}">
        <joint name="{side}_shoulder" axis="0 1 0" range="-180 90"/>
        <geom class="servo" type="box" pos="0 {_f(-sgn*SV_T/2)} 0" size="{_f(SV_WID/2)} {_f(SV_T/2)} {_f(SV_LEN/2)}" mass="{p.m_neck_servo}"/>
        <geom {_fc(p)}type="capsule" fromto="0 0 0 0 0 {_f(-p.arm_len)}" size="0.006" mass="{p.arm_mass}" rgba="0.82 0.84 0.87 1"/>
        <geom {_fc(p)}type="sphere" pos="0 0 {_f(-p.arm_len)}" size="0.012" mass="0.005" friction="1.0 0.02 0.001" rgba="0.2 0.2 0.2 1"/>
      </body>""")
    return "".join(out)


def _head(p: DesignParams) -> str:
    if not p.torso_v7:
        return ""
    zn = p.deck_bot + p.deck_t
    return f"""
      <body name="head" pos="0 0 {_f(zn + SV_T + 0.004)}">
        <joint name="neck_yaw" axis="0 0 1" range="-90 90"/>
        <geom {_fc(p)}type="box" pos="0.005 0 {_f(p.head_h/2)}" size="0.028 0.030 {_f(p.head_h/2)}" mass="{p.m_head}" rgba="0.82 0.84 0.87 1" group="1"/>
        <geom type="box" pos="0.036 0 {_f(p.head_h*0.6)}" size="0.005 0.012 0.012" mass="0.003" rgba="0.1 0.1 0.1 1" group="1"/>
        <site name="camera" pos="0.041 0 {_f(p.head_h*0.6)}" size="0.003" rgba="0 1 0 0.6"/>
      </body>"""


def build_xml(p: DesignParams) -> str:
    z0 = p.z_yaw_above_sole            # torso origin (yaw axis) above the floor
    d = p.deck_x
    deck_cx = (d[0] + d[1]) / 2
    deck_hx = (d[1] - d[0]) / 2
    zdeck = p.housing_h + p.deck_t / 2
    r = math.radians
    acts = []
    for side in ("L", "R"):
        roll_rng = ((-p.hip_roll_add, p.hip_roll_abd) if side == "L"
                    else (-p.hip_roll_abd, p.hip_roll_add))
        kr = p.knee_range
        hp = p.hip_pitch_range
        for jn, lo, hi in (
            ("hip_yaw", -p.yaw_range, p.yaw_range),
            ("hip_roll", roll_rng[0], roll_rng[1]),
            ("hip_pitch", hp[0], hp[1]),
            ("knee", kr[0], kr[1]),
            ("ankle", -p.ankle_range, p.ankle_range),
            ("ankle_roll", -p.ankle_roll_range, p.ankle_roll_range),
        ):
            acts.append(f'    <position name="{side}_{jn}" joint="{side}_{jn}" '
                        f'ctrlrange="{r(lo):.10f} {r(hi):.10f}"/>  <!-- {lo:.0f} .. {hi:.0f} -->')
    if p.torso_v7:
        acts.append(f'    <position name="neck_yaw" joint="neck_yaw" ctrlrange="{r(-90):.10f} {r(90):.10f}"/>  <!-- head yaw -->')
    if p.arms:
        for side in ("L", "R"):
            acts.append(f'    <position name="{side}_shoulder" joint="{side}_shoulder" ctrlrange="{r(-180):.10f} {r(90):.10f}"/>')
    acts_s = "\n".join(acts)
    pairs = []
    segs = ("col_thigh", "col_shank", "col_ankle", "col_foot")
    for a in segs:
        for b in segs:
            pairs.append(f'    <pair geom1="L_{a}" geom2="R_{b}"/>')
    pairs.append('    <pair geom1="L_sole" geom2="R_sole"/>')
    pairs_s = "\n".join(pairs)
    yaw_case = (SV_LEN / 2, SV_WID / 2, SV_T / 2)
    return f"""<mujoco model="bimo_biped_v6ar">
  <!-- GENERATED by sim/gen_plant_v6.py -- edit the generator, not this file.
       v6 body: 6 DOF/leg with ANKLE ROLL, {p.summary()}.
       See sim/gen_plant_v6.py for the mass model and docs/design-v6-ankle-roll.md
       for the design record. -->
  <compiler angle="degree" autolimits="true"/>
  <option timestep="0.002" gravity="0 0 -9.81" integrator="implicitfast"/>

  <visual>
    <global offwidth="900" offheight="1100"/>
    <headlight diffuse="0.7 0.7 0.7" ambient="0.35 0.35 0.35"/>
  </visual>

  <default>
    <joint type="hinge" damping="0.6" armature="0.008" frictionloss="0.02" limited="true"/>
    <position kp="40" forcerange="-3 3"/>
    <geom contype="0" conaffinity="0"/>
    <default class="servo">
      <geom type="box" contype="2" conaffinity="0" group="1" rgba="0.22 0.23 0.27 1"/>
    </default>
    <default class="pad">
      <geom type="sphere" size="0.003" contype="1" conaffinity="0" mass="0"
            friction="1 0.02 0.001" condim="4" rgba="0.20 0.21 0.24 1"/>
    </default>
    <default class="fallcol">
      <geom contype="2" conaffinity="0" group="1" friction="0.8 0.02 0.001" condim="4"/>
    </default>
    <default class="legcol">
      <geom type="capsule" size="0.0147" contype="0" conaffinity="0" mass="0"
            group="4" rgba="0.85 0.35 0.25 0.30"/>
    </default>
  </default>

  <asset>
    <texture name="grid" type="2d" builtin="checker" rgb1="0.87 0.89 0.92"
             rgb2="0.77 0.80 0.85" width="512" height="512"/>
    <material name="floor_mat" texture="grid" texrepeat="24 24" texuniform="true"/>
  </asset>

  <worldbody>
    <light pos="0.6 0.4 1.5" dir="-0.4 -0.3 -1" directional="true"/>
    <geom name="floor" type="plane" size="3 3 0.1" contype="1" conaffinity="3"
          material="floor_mat" friction="1 0.02 0.001" condim="4"/>

    <!-- torso origin = hip yaw axis height, on the centreline -->
    <body name="torso" pos="0 0 {_f(z0)}">
      <freejoint/>
{_torso(p)}
{_leg(p, "L")}
{_leg(p, "R")}{_head(p)}{_arms(p)}
    </body>
  </worldbody>

  <actuator>
{acts_s}
  </actuator>

  <contact>
{pairs_s}
  </contact>

  <sensor>
    <framezaxis objtype="site" objname="imu" name="torso_up"/>
    <framepos   objtype="site" objname="imu" name="torso_pos"/>
  </sensor>
</mujoco>
"""


def params_from_args(argv=None) -> tuple[DesignParams, argparse.Namespace]:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-o", "--out", default=DEFAULT_OUT)
    ap.add_argument("--knee", choices=("fwd", "bwd"), default=None)
    ap.add_argument("--torso-v7", action="store_true", help="Pi 4B torso + neck servo + head")
    ap.add_argument("--servo", choices=("sts3215", "sts3250"), default=None,
                    help="servo mass everywhere (case is identical)")
    ap.add_argument("--set", nargs="*", default=[], metavar="KEY=VAL",
                    help="override any DesignParams field")
    a = ap.parse_args(argv)
    p = DesignParams()
    if a.knee:
        p.knee = a.knee
    if a.torso_v7:
        p.torso_v7 = True
    if a.servo == "sts3250":
        p.servo_mass = 0.0745
    for kv in a.set:
        k, v = kv.split("=", 1)
        cur = getattr(p, k)
        if isinstance(cur, bool):
            setattr(p, k, v.lower() in ("1", "true", "yes"))
        elif isinstance(cur, tuple):
            setattr(p, k, tuple(float(x) for x in v.split(",")))
        elif isinstance(cur, str):
            setattr(p, k, v)
        else:
            setattr(p, k, type(cur)(v))
    return p, a


def main(argv=None):
    p, a = params_from_args(argv)
    xml = build_xml(p)
    with open(a.out, "w") as f:
        f.write(xml)
    import mujoco
    m = mujoco.MjModel.from_xml_path(a.out)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    print(f"wrote {a.out}")
    print(f"  {p.summary()}")
    print(f"  nq {m.nq} nv {m.nv} nu {m.nu}  total mass {m.body_subtreemass[0]:.3f} kg  "
          f"CoM z {d.subtree_com[0][2]:.3f} m")
    # lowest pad
    zmin = min(d.geom_xpos[g][2] - m.geom_size[g][0] for g in range(m.ngeom)
               if (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or "").find("_pad_") > 0)
    print(f"  lowest pad z {zmin:+.4f} (should be ~0)")


if __name__ == "__main__":
    main()
