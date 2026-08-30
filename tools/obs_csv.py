"""Sim-side writer of the firmware's obsdump CSV format.

The firmware's obsdump (firmware/main/obs_dump.cpp) streams one CSV record
per tick: `OBSHDR,<names>` once, then `OBS,<tick>,<83 values>`. This module
reproduces those column names and rows for a SIM capture, so
tools/obs_capture.py --compare can put a bench capture and a sim capture side
by side without either tool keeping a second copy of the layout.

Names are generated from the same facts the firmware generates them from
(joint order, cmd channel order, frame offsets) -- passed in from the SIL
lib's spec, which the probe guarantees matches the compiled firmware.
"""

CMD_NAMES = ["vx", "vy", "wz", "crouch", "lift", "foot_dx", "foot_dz"]
AXES = ["x", "y", "z"]


def columns(joint_names, frame_offsets, frame_dim, num_cmd=7):
    """Column names, exactly as firmware obs_dump.cpp emits them."""
    def cmd_name(i):
        return CMD_NAMES[i] if i < len(CMD_NAMES) else f"c{i}"

    cols = ["tick"]
    cols += [f"q_{j}" for j in joint_names]
    cols += [f"dq_{j}" for j in joint_names]
    cols += [f"up_{a}" for a in AXES]
    cols += [f"gyro_{a}" for a in AXES]
    cols += ["phase"]
    cols += [f"cmd_{cmd_name(i)}" for i in range(num_cmd)]

    o = frame_offsets
    for i in range(frame_dim):
        if i < o["dq"]:
            cols.append(f"f_q_{joint_names[i - o['q']]}")
        elif i < o["up"]:
            cols.append(f"f_dq_{joint_names[i - o['dq']]}")
        elif i < o["linvel"]:
            cols.append(f"f_up_{AXES[i - o['up']]}")
        elif i < o["gyro"]:
            cols.append(f"f_linvel_{AXES[i - o['linvel']]}")
        elif i < o["prev_action"]:
            cols.append(f"f_gyro_{AXES[i - o['gyro']]}")
        elif i < o["height"]:
            cols.append(f"f_pa_{joint_names[i - o['prev_action']]}")
        elif i < o["phase"]:
            cols.append("f_height")
        elif i < o["cmd"]:
            cols.append("f_sin" if i - o["phase"] == 0 else "f_cos")
        else:
            cols.append(f"f_cmd_{cmd_name(i - o['cmd'])}")
    return cols


class Writer:
    """OBSHDR/OBS lines into a file, firmware-format."""

    def __init__(self, path, joint_names, frame_offsets, frame_dim,
                 num_cmd=7):
        self.f = open(path, "w")
        self.cols = columns(joint_names, frame_offsets, frame_dim, num_cmd)
        self.f.write("OBSHDR," + ",".join(self.cols) + "\r\n")
        self.n = 0

    def row(self, tick, values):
        """values: iterable of len(cols)-1 floats (everything after tick)."""
        vals = ",".join(f"{float(v):.6g}" for v in values)
        self.f.write(f"OBS,{int(tick)},{vals}\r\n")
        self.n += 1

    def close(self):
        self.f.close()
