"""The bought parts as solids, for the interference checks and the assembly:
the ST3215 servo in its three mounted orientations, the General Driver board
(PCB, plugs, component side) and the GoPro.

    servo_mock(idler_disc=True)   the servo in its one canonical frame
    servo_mock_y / _x / _z        rigid rotations of it (pitch, roll, yaw)

Conservative by design: where a mock and the real part differ, the mock is
the bigger one.
"""
from build123d import *  # noqa: F401,F403

from common import dimensions as D
from common import shapes


def servo_mock(idler_disc=True):
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
    s = shapes.box(-D.SV_WID / 2, D.SV_WID / 2,
                  D.SV_IDLER_CASE_FACE, D.SV_TOPFACE,
                  -D.SV_AXIS_FROM_REAR, D.SV_AXIS_FROM_OUT_END)
    # horn-side rib, on the cable half (cable end is -Z in this frame)
    s += shapes.box(-D.SV_HORN_RIB_HW, D.SV_HORN_RIB_HW,
                   D.SV_TOPFACE, D.SV_TOPFACE + D.SV_HORN_RIB_H,
                   -D.SV_HORN_RIB_L[1], -D.SV_HORN_RIB_L[0])
    # moulded back-cover platform, proud of the idler case face
    s += shapes.box(-D.SV_IDLER_BOSS_HW, D.SV_IDLER_BOSS_HW,
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
    # idler_disc=False: a servo built without its idler disc (v6's hip-yaw
    # servos); the free-hub post stays, it is part of the servo.
    if idler_disc:
        s += shapes.cyl_y(D.SV_DISC_R, D.SV_IDLER_FACE, D.SV_IDLER_CASE_FACE, 0, 0)
    s += shapes.cyl_y(D.SV_IDLER_HUB_HW, D.SV_BOTFACE, D.SV_IDLER_CASE_FACE, 0, 0)
    s += shapes.cyl_y(D.SV_BOSS_D / 2, D.SV_TOPFACE, D.SV_HORN_FACE, 0, 0)
    return s


def servo_mock_y():
    """Leg pitch joints (hip pitch, knee) and -- rolled 90 deg -- the ankle.
    Output axis +Y, body length along Z, output end +10.11, cable end -35.11."""
    return servo_mock()


def servo_mock_x():
    """Roll servo: output axis +X (horn +X), body length along Z with the
    output end DOWN -- cable end +35.11 at the top, output end -10.11."""
    return Rot(0, 180, 0) * Rot(0, 0, 90) * servo_mock()


def servo_mock_z(idler_disc=True):
    """Yaw servo: output axis VERTICAL with the horn DOWN. Case length
    along X (YAW_CASE_X_REAR..FRONT), width along Y, idler face UP."""
    return Rot(0, 90, 0) * Rot(0, 0, -90) * servo_mock(idler_disc)


# ------------------------------------------------------------- torso mocks
# The bought parts the v6 torso holds: the driver board, its component side, and
# the camera.
def board_pcb_mock():
    """The General Driver PCB seated in the v6 recess, pelvis frame. CORRECTED
    outline 65.01 x 56.01 (the wiki's 65 x 65 was 8.99 mm of board that does not
    exist -- BOARD_GD_OUTLINE), standing upright and transverse with its 65.01
    BUS EDGE UP: forward face on the standoffs at BR_PCB_X0, y +/-32.505,
    z BR_BOT_Z..BR_TOP_Z, so only the top 15 mm clears the deck.

    THE TWO USB-C SHELLS ARE MODELLED, because they are the whole reason this
    mock was lying (2026-08-06). They stand BOARD_GD_SHELL_PROUD off the +y
    service edge -- 0.91, against 0.5 of cheek slip -- so a mock that stopped at
    the outline read "clear" on a board that physically does not go in.

    ITS OWN MOUNTING HOLES ARE MODELLED (BOARD_GD_HOLE_D on the BOARD_GD_HOLES
    grid). Without them a solid slab reads as blocking its own screws -- the
    kind of false positive that gets explained away in a comment once and then
    hides a real one later."""
    hy = D.BOARD_GD_OUTLINE[0] / 2
    p = shapes.box(D.BR_PCB_X1, D.BR_PCB_X0, -hy, hy, D.BR_BOT_Z, D.BR_TOP_Z)
    for sy in (D.BOARD_GD_SCREW_DY, -D.BOARD_GD_SCREW_DY):
        for sz in (D.BR_SCREW_Z, D.BR_UPPER_ROW_Z):
            p -= shapes.cyl_x(D.BOARD_GD_HOLE_D / 2, D.BR_PCB_X1 - 1,
                             D.BR_PCB_X0 + 1, sy, sz)
    for z0, z1 in (D.BR_USB_Z, D.BR_LIDAR_Z):     # the receptacles, 3.26 tall
        p += shapes.box(D.BR_PCB_X1 - 3.26, D.BR_PCB_X1,
                       D.BR_SVC_SIGN * hy, D.BR_SVC_SIGN * D.BR_SHELL_Y, z0, z1)
    return p


def board_plugs_mock():
    """What is MATED to the service edge in service: the USB-C flashing lead and
    the battery pigtail, as their overmoulded plug bodies (USBC_PLUG, XH2_PLUG)
    sitting outboard of the board and running out through the cheek. No slip
    added -- PLUG_CLR is already in the cut, so this is the honest body and any
    overlap the checker reports is real material in the way.

    The LiDAR port gets no plug: nothing is fitted to it, which is exactly why
    its shell gets a relief groove and not a window."""
    s = D.BR_SVC_SIGN
    y0 = D.BR_SHELL_Y                        # the plug starts at the shell face
    out = None
    for (z0, z1), (u_len, thick, reach) in ((D.BR_USB_Z, D.USBC_PLUG),
                                            (D.BR_INLET_Z, D.XH2_PLUG)):
        cz = (z0 + z1) / 2
        b = shapes.box(D.BR_SVC_PORT_CX - thick / 2, D.BR_SVC_PORT_CX + thick / 2,
                      s * y0, s * (y0 + reach),
                      cz - u_len / 2, cz + u_len / 2)
        out = b if out is None else out + b
    return out


def board_comp_mock():
    """Everything fitted to the board's AFT face, as ONE conservative slab of
    BOARD_GD_COMP -- the 40-pin header is the tall one, and the recess cheeks
    are sized against it (BR_RIM_PROUD)."""
    hy = D.BOARD_GD_OUTLINE[0] / 2
    return shapes.box(D.BR_COMP_X, D.BR_PCB_X1, -hy, hy, D.BR_BOT_Z, D.BR_TOP_Z)


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
    body = shapes.box(-dx / 2, dx / 2, -dy / 2, dy / 2, bot, bot + dz)
    fingers = None
    for cy in (-(D.GP_PRONG_T + D.GP_SLOT) / 2, (D.GP_PRONG_T + D.GP_SLOT) / 2):
        f = shapes.box(-6, 6, cy - 1.45, cy + 1.45, hole_z - 6, bot + 1)
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
