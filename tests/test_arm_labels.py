"""The arm links' side letters (cad/parts.py letter_y, cad/v6/arm_v6.py): each
link's L or R sits wholly in its outboard rail and reads correctly from
outside, upright as the arm hangs. A letter cut before the mirror reads
backwards on the right arm, and fails here."""
import os
import sys

import pytest
from build123d import Vector

HERE = os.path.dirname(os.path.abspath(__file__))
for p in ("..", "../cad", "../cad/v6"):
    sys.path.insert(0, os.path.join(HERE, p))

import arm_v6 as A  # noqa: E402
import parts as P  # noqa: E402

V = A.V
LINKS = {"arm_upper_v6": (A.arm_upper_v6, V.ARM_SHAFT_Y[1], V.ARM_LABEL_Z_UPPER),
         "arm_fore_v6": (A.arm_fore_v6, V.ARM_FORE_SHAFT_HY, V.ARM_LABEL_Z_FORE)}


@pytest.mark.parametrize("ch", "LR")
def test_letter_is_one_face_inside_its_box(ch):
    f = P.letter_2d(ch, 8.0)
    bb = f.bounding_box()
    assert len(f.faces()) == 1 and f.faces()[0].normal_at().Z > 0.99
    assert bb.size.Y == pytest.approx(8.0) and 4.0 < bb.size.X < 6.0
    assert abs(bb.center().X) < 1e-6 and abs(bb.center().Y) < 1e-6


@pytest.mark.parametrize("outward", (1, -1))
def test_letter_cuts_depth_into_the_face(outward):
    c = P.letter_y("L", outward * 10.0, outward, 0, 0, 8.0, 0.6)
    lo, hi = sorted((outward * 10.0, outward * (10.0 - 0.6)))
    bb = c.bounding_box()
    assert (bb.min.Y, bb.max.Y) == pytest.approx((lo, hi + 0.2) if outward > 0 else (lo - 0.2, hi))


def _seen_from_outside(o, x, z, u, v):
    """Model point at the face for letter coordinates (u right, v up) as a
    viewer outboard of it sees them: right is -o * x, up is +z."""
    return x - o * u, z + v


@pytest.mark.parametrize("link", sorted(LINKS))
@pytest.mark.parametrize("side", "LR")
def test_link_letter_is_on_its_outboard_rail_and_reads_from_outside(link, side):
    fn, y_face, z = LINKS[link]
    o = 1 if side == "L" else -1
    part = fn(side)
    cut = P.letter_y(side, o * y_face, o, V.ARM_LABEL_X, z, V.ARM_LABEL_H, V.ARM_LABEL_DEPTH)
    in_part = cut.volume * V.ARM_LABEL_DEPTH / (V.ARM_LABEL_DEPTH + 0.2)
    # the letter took exactly its own volume: wholly in the rail, nothing spilled past it
    restored = part + (cut & P.box(-50, 50, -y_face, y_face, z - 10, z + 10))
    assert restored.volume - part.volume == pytest.approx(in_part, rel=1e-3)
    # read from outside, 0.3 mm into the groove: the letter's own corners
    h, y = V.ARM_LABEL_H, o * (y_face - 0.3)
    hw = 0.3 * h if side == "L" else 0.34 * h
    ink = [(-hw + 0.3, h / 2 - 0.3), (-hw + 0.3, -h / 2 + 0.3)]          # the stem, top and bottom
    bare = [(hw - 0.3, h / 2 - 0.3)]                     # top right: past the L's foot, outside the R's bowl
    if side == "R":
        bare += [(hw - 0.28 * h, h / 2 - 0.28 * h)]      # the R's counter, the bowl's centre
    ink += [(hw - 0.3, -h / 2 + 0.3)]                                    # L's foot, R's leg: bottom right
    for (u, v), want in [(p, True) for p in ink] + [(p, False) for p in bare]:
        px, pz = _seen_from_outside(o, V.ARM_LABEL_X, z, u, v)
        assert part.is_inside(Vector(px, y, pz)) is not want, (side, u, v)
