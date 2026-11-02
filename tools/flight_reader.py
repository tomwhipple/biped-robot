#!/usr/bin/env python3
"""Read a flight-recorder session (.frec) and line it up with everything else.

    tools/flight_reader.py summary  S.frec
    tools/flight_reader.py csv      S.frec [--parquet]       # S_ticks.csv, S_events.csv
    tools/flight_reader.py align    S.frec --pts V.pts [--serial L.log] [--date YYYY-MM-DD]
    tools/flight_reader.py plot     S.frec [--joints R_knee,L_knee] [--pts V.pts]
                                    [--serial L.log] [--t0 HH:MM:SS] [--t1 ...]

Time. Every packet carries the robot's NTP-synced UTC (header t_utc_us) and
its boot clock (t_boot_us). Packets sent before the first SNTP reply have no
UTC; they get one from the boot->UTC offset of the synced packets of the same
boot, or failing that from the host receive time (the minimum of rx - boot is
the latency floor). Each tick also says when its servo read happened
(fb_off_us) -- "present position" belongs to THAT instant, not the tick start.

Sony video .pts files (tools/sony_record.sh, cam_record.sh) are one epoch-ms
per frame after a "# timecode format v2" line; serial logs are
"HH:MM:SS.mmmZ <text>" lines ("> cmd" for what was typed), dated by --date or
by the YYYY-MM-DD directory they sit in.

Angles: steps -> rad with the calibration in the session's own HELLO packets
(zero, dir per bus joint), i.e. obs::busStepsToAngle -- the joint frame the
sim uses.
"""
from __future__ import annotations

import argparse
import datetime as dt
import json
import os
import re
import struct
import sys
from dataclasses import dataclass, field

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import flightrec as F  # noqa: E402

FILE_MAGIC = b"FRECSESS1\n"
REC = struct.Struct("<HQ")


@dataclass
class Session:
    path: str
    meta: dict
    packets: list = field(default_factory=list)   # decoded dicts + rx_us
    undecodable: int = 0

    @property
    def ticks(self):
        return [p for p in self.packets if p["type"] == F.T_TICK]

    @property
    def events(self):
        return [p for p in self.packets if p["type"] == F.T_EVENT]

    @property
    def hellos(self):
        return [p for p in self.packets if p["type"] == F.T_HELLO]


def load(path: str) -> Session:
    with open(path, "rb") as f:
        if f.read(len(FILE_MAGIC)) != FILE_MAGIC:
            raise ValueError(f"{path}: not a flight session file")
        meta = json.loads(f.readline())
        s = Session(path, meta)
        while True:
            hdr = f.read(REC.size)
            if len(hdr) < REC.size:
                break
            n, rx_us = REC.unpack(hdr)
            data = f.read(n)
            if len(data) < n:
                break                       # truncated tail (collector killed)
            try:
                p = F.decode(data)
            except F.DecodeError:
                s.undecodable += 1
                continue
            p["rx_us"] = rx_us
            s.packets.append(p)
    _assign_utc(s)
    s.packets.sort(key=lambda p: (p["utc_us"], p["seq"]))
    return s


def _assign_utc(s: Session) -> None:
    """utc_us on every packet: its own stamp, else boot offset, else rx."""
    off_sync: dict[int, list[int]] = {}
    off_rx: dict[int, int] = {}
    for p in s.packets:
        b = p["boot_id"]
        if p["t_utc_us"]:
            off_sync.setdefault(b, []).append(p["t_utc_us"] - p["t_boot_us"])
        o = p["rx_us"] - p["t_boot_us"]
        off_rx[b] = min(off_rx.get(b, o), o)
    med = {b: sorted(v)[len(v) // 2] for b, v in off_sync.items()}
    for p in s.packets:
        if p["t_utc_us"]:
            p["utc_us"], p["utc_src"] = p["t_utc_us"], "robot"
        elif p["boot_id"] in med:
            p["utc_us"], p["utc_src"] = p["t_boot_us"] + med[p["boot_id"]], "boot+ntp"
        else:
            p["utc_us"], p["utc_src"] = p["t_boot_us"] + off_rx[p["boot_id"]], "host_rx"


def calibration(s: Session) -> dict[int, dict]:
    """boot_id -> {bus: (zero, dir, id, fitted)} from the latest HELLO."""
    cal = {}
    for h in s.hellos:
        cal[h["boot_id"]] = {b: (h["zero"][b], h["dir"][b], h["id"][b], h["fitted"][b])
                             for b in range(F.MAX_JOINTS)}
    return cal


# -- tables ------------------------------------------------------------------
JOINT_COLS = ("nominal", "goal", "goal_speed", "pos", "speed", "load", "current",
              "voltage_dv", "temp_c", "status", "moving", "flags")


def tick_rows(s: Session) -> list[dict]:
    """One flat dict per tick: header, IMU, balance, ctrl, <joint>.<field>.

    Per joint: the raw steps, plus .nom_rad/.goal_rad/.pos_rad in the joint
    frame and .err_rad = pos - goal (NaN when either is invalid).
    """
    cal = calibration(s)
    nan = float("nan")
    rows = []
    for p in s.ticks:
        r = dict(seq=p["seq"], boot=f"{p['boot_id']:08x}", t_utc=p["utc_us"] / 1e6,
                 utc_src=p["utc_src"], t_boot=p["t_boot_us"] / 1e6,
                 t_fb_utc=(p["utc_us"] + p["fb_off_us"]) / 1e6,
                 rx_utc=p["rx_us"] / 1e6, source=F.SOURCE_NAMES.get(p["source"], p["source"]),
                 flags=p["flags"], link_state=p["link_state"], tick=p["tick"],
                 work_us=p["work_us"], fb_read_us=p["fb_read_us"],
                 imu_age_us=p["imu_age_us"], drop_ring=p["drop_ring"],
                 drop_net=p["drop_net"], roll=p["roll"], pitch=p["pitch"])
        for i, ax in enumerate("xyz"):
            r[f"accel_{ax}"] = p["accel"][i]
            r[f"gyro_{ax}"] = p["gyro"][i]
            r[f"up_{ax}"] = p["up"][i]
        for i, c in enumerate("wxyz"):
            r[f"q{c}"] = p["quat"][i]
        b = p["bal"]
        r.update({f"bal_{k}": v for k, v in b.items()})
        r["bal_state_name"] = F.BAL_STATES.get(b["state"], str(b["state"]))
        c = p["ctrl"]
        for i in range(F.NUM_CMD):
            r[f"cmd{i}"] = c["cmd"][i]
        r.update(phase=c["phase"], vbat_mv=c["vbat_mv"], batt=c["batt"],
                 home_stage=c["home_stage"])
        bcal = cal.get(p["boot_id"], {})
        for j in p["joints"]:
            name = F.BUS_JOINT_NAMES[j["bus"]] if j["bus"] < len(F.BUS_JOINT_NAMES) else f"bus{j['bus']}"
            for k in JOINT_COLS:
                r[f"{name}.{k}"] = j[k]
            fl = j["flags"]
            zero, d = (bcal.get(j["bus"]) or (2048, 1, 0, 0))[:2]
            nom = F.steps_to_rad(j["nominal"], zero, d) if fl & F.JF_NOM_VALID else nan
            goal = F.steps_to_rad(j["goal"], zero, d) if fl & F.JF_GOAL_VALID else nan
            pos = F.steps_to_rad(j["pos"], zero, d) if fl & F.JF_FB_OK else nan
            r[f"{name}.nom_rad"] = nom
            r[f"{name}.goal_rad"] = goal
            r[f"{name}.pos_rad"] = pos
            r[f"{name}.err_rad"] = pos - goal
        rows.append(r)
    return rows


def event_rows(s: Session) -> list[dict]:
    out = []
    for p in s.events:
        out.append(dict(seq=p["seq"], t_utc=p["utc_us"] / 1e6, utc_src=p["utc_src"],
                        kind=F.EVENT_NAMES.get(p["code"], str(p["code"])),
                        src="ctrl" if p["src"] == 1 else "house",
                        a=p["a"], b=p["b"], text=p["text"]))
    return out


def summary(s: Session) -> dict:
    ticks = s.ticks
    by_src: dict[str, list] = {}
    for p in ticks:
        by_src.setdefault(F.SOURCE_NAMES.get(p["source"], str(p["source"])), []).append(p)
    out = dict(file=os.path.basename(s.path), packets=len(s.packets),
               undecodable=s.undecodable, ticks=len(ticks), events=len(s.events),
               hellos=len(s.hellos), sources={})
    if s.packets:
        out["start_utc"] = _iso(s.packets[0]["utc_us"])
        out["end_utc"] = _iso(s.packets[-1]["utc_us"])
    for name, ps in by_src.items():
        ts = sorted(p["t_boot_us"] for p in ps)
        dts = sorted((b - a) / 1e6 for a, b in zip(ts, ts[1:]) if 0 < b - a < 2e6)
        fb_ok: dict[str, list[int]] = {}
        for p in ps:
            for j in p["joints"]:
                nm = F.BUS_JOINT_NAMES[j["bus"]]
                c = fb_ok.setdefault(nm, [0, 0])
                c[1] += 1
                c[0] += bool(j["flags"] & F.JF_FB_OK)
        reads = [p["fb_read_us"] for p in ps if p["flags"] & F.TF_FB_READ]
        out["sources"][name] = dict(
            ticks=len(ps),
            median_hz=round(1 / dts[len(dts) // 2], 2) if dts else None,
            period_ms_p05_p95=[round(dts[int(0.05 * (len(dts) - 1))] * 1e3, 1),
                               round(dts[int(0.95 * (len(dts) - 1))] * 1e3, 1)] if dts else None,
            fb_read_us_median=sorted(reads)[len(reads) // 2] if reads else None,
            feedback_answered_pct={k: round(100 * v[0] / v[1], 1) for k, v in fb_ok.items()})
    # holes, per boot
    seqs: dict[int, list[int]] = {}
    for p in s.packets:
        seqs.setdefault(p["boot_id"], []).append(p["seq"])
    out["boots"] = {}
    for b, sq in seqs.items():
        sq = sorted(set(sq))
        missing = (sq[-1] - sq[0] + 1) - len(sq)
        last = max((p for p in s.packets if p["boot_id"] == b), key=lambda p: p["seq"])
        first = min((p for p in s.packets if p["boot_id"] == b), key=lambda p: p["seq"])
        out["boots"][f"{b:08x}"] = dict(
            seq_first=sq[0], seq_last=sq[-1], missing=missing,
            robot_drop_ring=last["drop_ring"] - first["drop_ring"],
            robot_drop_net=last["drop_net"] - first["drop_net"])
    out["utc_sources"] = {k: sum(1 for p in s.packets if p["utc_src"] == k)
                          for k in ("robot", "boot+ntp", "host_rx")}
    return out


def _iso(us: int) -> str:
    return dt.datetime.fromtimestamp(us / 1e6, dt.timezone.utc).isoformat(timespec="milliseconds")


# -- the other clocks --------------------------------------------------------
def read_pts(path: str) -> list[int]:
    """Epoch ms per video frame."""
    out = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line and not line.startswith("#"):
                out.append(int(float(line)))
    return out


_SERIAL = re.compile(r"^(\d\d):(\d\d):(\d\d)\.(\d{3})Z (.*)$")


def read_serial_log(path: str, date: str | None = None) -> list[dict]:
    """[{t_utc, kind: 'cmd'|'out'|'note', text}] from an HH:MM:SS.mmmZ log."""
    if date is None:
        m = re.search(r"(\d{4}-\d\d-\d\d)", os.path.abspath(path))
        if not m:
            raise ValueError("serial log date unknown: pass --date YYYY-MM-DD")
        date = m.group(1)
    day = dt.datetime.strptime(date, "%Y-%m-%d").replace(tzinfo=dt.timezone.utc)
    out = []
    prev = None
    roll = 0
    with open(path, errors="replace") as f:
        for line in f:
            m = _SERIAL.match(line.rstrip("\n"))
            if not m:
                continue
            h, mi, se, ms, text = m.groups()
            t = int(h) * 3600 + int(mi) * 60 + int(se) + int(ms) / 1000
            if prev is not None and t + roll * 86400 < prev - 3600:
                roll += 1                     # the log ran past midnight UTC
            t += roll * 86400
            prev = t
            kind = "cmd" if text.startswith("> ") else ("note" if text.startswith("# ") else "out")
            out.append(dict(t_utc=day.timestamp() + t, kind=kind,
                            text=text[2:] if kind != "out" else text.strip()))
    return out


def nearest(sorted_t: list[float], t: float) -> int:
    """Index of the value in sorted_t closest to t."""
    import bisect
    i = bisect.bisect_left(sorted_t, t)
    if i == 0:
        return 0
    if i >= len(sorted_t):
        return len(sorted_t) - 1
    return i if sorted_t[i] - t < t - sorted_t[i - 1] else i - 1


def align_video(rows: list[dict], pts_ms: list[int], max_dt_s: float = 0.1) -> list[dict]:
    """Per video frame: the tick nearest in time (by the servo-read instant)."""
    ts = [r["t_fb_utc"] for r in rows]
    out = []
    for k, ms in enumerate(pts_ms):
        t = ms / 1000.0
        row = dict(frame=k, frame_utc=t)
        if ts:
            i = nearest(ts, t)
            d = ts[i] - t
            row.update(tick_seq=rows[i]["seq"], tick_dt_ms=round(d * 1e3, 2),
                       in_record=abs(d) <= max_dt_s)
        out.append(row)
    return out


# -- outputs -----------------------------------------------------------------
def write_csv(rows: list[dict], path: str) -> None:
    import csv
    cols: list[str] = []
    seen = set()
    for r in rows:
        for k in r:
            if k not in seen:
                seen.add(k)
                cols.append(k)
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        for r in rows:
            w.writerow(r)


def stem(path: str) -> str:
    return path[:-5] if path.endswith(".frec") else path


def cmd_csv(a) -> int:
    s = load(a.session)
    ticks, events = tick_rows(s), event_rows(s)
    write_csv(ticks, stem(a.session) + "_ticks.csv")
    write_csv(events, stem(a.session) + "_events.csv")
    print(f"{stem(a.session)}_ticks.csv ({len(ticks)} rows), _events.csv ({len(events)} rows)")
    if a.parquet:
        try:
            import pandas as pd
            pd.DataFrame(ticks).to_parquet(stem(a.session) + "_ticks.parquet")
            print(f"{stem(a.session)}_ticks.parquet")
        except ImportError as e:
            print(f"parquet skipped: {e} (pip install pyarrow)")
    return 0


def cmd_summary(a) -> int:
    print(json.dumps(summary(load(a.session)), indent=1))
    return 0


def cmd_align(a) -> int:
    s = load(a.session)
    rows = tick_rows(s)
    base = stem(a.session)
    if a.pts:
        al = align_video(rows, read_pts(a.pts))
        out = base + "_" + os.path.basename(a.pts).rsplit(".", 1)[0] + "_frames.csv"
        write_csv(al, out)
        inside = sum(1 for r in al if r.get("in_record"))
        print(f"{out}: {len(al)} frames, {inside} within 100 ms of a tick")
    timeline = [dict(t_utc=e["t_utc"], origin="robot", kind=e["kind"], text=e["text"])
                for e in event_rows(s)]
    if a.serial:
        timeline += [dict(t_utc=e["t_utc"], origin="serial", kind=e["kind"], text=e["text"])
                     for e in read_serial_log(a.serial, a.date)]
    timeline.sort(key=lambda r: r["t_utc"])
    for r in timeline:
        r["utc"] = _iso(int(r["t_utc"] * 1e6))
    write_csv(timeline, base + "_timeline.csv")
    print(f"{base}_timeline.csv: {len(timeline)} events (robot + serial)")
    return 0


def _clock(t: str, day: float) -> float:
    h, m, sec = t.split(":")
    return day + int(h) * 3600 + int(m) * 60 + float(sec)


def cmd_plot(a) -> int:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    s = load(a.session)
    rows = tick_rows(s)
    if not rows:
        print("no ticks in this session")
        return 1
    day = dt.datetime.fromtimestamp(rows[0]["t_utc"], dt.timezone.utc).replace(
        hour=0, minute=0, second=0, microsecond=0).timestamp()
    lo = _clock(a.t0, day) if a.t0 else rows[0]["t_utc"]
    hi = _clock(a.t1, day) if a.t1 else rows[-1]["t_utc"]
    rows = [r for r in rows if lo <= r["t_utc"] <= hi]
    if not rows:
        print("no ticks in that window")
        return 1
    t0 = rows[0]["t_utc"]
    joints = a.joints.split(",") if a.joints else [
        n for n in F.BUS_JOINT_NAMES if any(f"{n}.pos_rad" in r for r in rows)][:12]
    deg = 57.29578
    # fixed categorical order (dataviz palette slots 1-3): goal, present, nominal
    c_goal, c_pos, c_nom, ink = "#2a78d6", "#eb6834", "#8a8a85", "#3d3d3a"
    ncol = 3
    nrow_j = (len(joints) + ncol - 1) // ncol
    fig = plt.figure(figsize=(15, 2.1 * nrow_j + 5.2))
    gs = fig.add_gridspec(nrow_j + 2, ncol, hspace=0.55, wspace=0.22)
    marks = [(e["t_utc"], e["text"]) for e in event_rows(s)
             if e["kind"] == "cmd" and lo <= e["t_utc"] <= hi]
    if a.serial:
        marks += [(e["t_utc"], e["text"]) for e in read_serial_log(a.serial, a.date)
                  if e["kind"] == "cmd" and lo <= e["t_utc"] <= hi]
    video = None
    if a.pts:
        p = read_pts(a.pts)
        if p:
            video = (p[0] / 1000 - t0, p[-1] / 1000 - t0)

    def deco(ax):
        for t, _ in marks:
            ax.axvline(t - t0, color=ink, lw=0.6, alpha=0.35)
        if video:
            ax.axvspan(*video, color="#2a78d6", alpha=0.05, lw=0)
        ax.grid(True, lw=0.4, alpha=0.4)
        ax.tick_params(labelsize=7)
        for sp in ("top", "right"):
            ax.spines[sp].set_visible(False)

    for k, name in enumerate(joints):
        ax = fig.add_subplot(gs[k // ncol, k % ncol])
        tt = [r["t_fb_utc"] - t0 for r in rows]
        for col, c, ls, lab in ((".nom_rad", c_nom, "--", "nominal"),
                                (".goal_rad", c_goal, "-", "goal written"),
                                (".pos_rad", c_pos, "-", "present")):
            ys = [r.get(name + col, float("nan")) * deg for r in rows]
            ax.plot(tt, ys, color=c, lw=1.4 if col != ".nom_rad" else 1.0, ls=ls, label=lab)
        ax.set_title(name + " (deg)", fontsize=8, color=ink, loc="left")
        deco(ax)
        if k == 0:
            ax.legend(fontsize=7, frameon=False, loc="best")
    tt = [r["t_utc"] - t0 for r in rows]
    ax = fig.add_subplot(gs[nrow_j, :])
    ax.plot(tt, [r["roll"] * deg for r in rows], color=c_goal, lw=1.4, label="torso roll")
    ax.plot(tt, [r["pitch"] * deg for r in rows], color=c_pos, lw=1.4, label="torso pitch")
    ax.set_title("IMU attitude (deg, ZYX from the fused quaternion)", fontsize=8, loc="left")
    ax.legend(fontsize=7, frameon=False, loc="upper left")
    deco(ax)
    ax = fig.add_subplot(gs[nrow_j + 1, :])
    ax.plot(tt, [r["bal_err"] * deg for r in rows], color=c_goal, lw=1.4, label="balance err")
    ax.plot(tt, [r["bal_u"] * deg for r in rows], color=c_pos, lw=1.4, label="balance u")
    ax.set_title("stance balance (deg); vertical lines = commands", fontsize=8, loc="left")
    ax.set_xlabel(f"s after {_iso(int(t0 * 1e6))}", fontsize=8)
    ax.legend(fontsize=7, frameon=False, loc="upper left")
    deco(ax)
    out = a.out or stem(a.session) + ".png"
    fig.savefig(out, dpi=110, bbox_inches="tight", facecolor="white")
    print(out)
    return 0


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = p.add_subparsers(dest="cmd", required=True)
    q = sub.add_parser("summary")
    q.add_argument("session")
    q.set_defaults(fn=cmd_summary)
    q = sub.add_parser("csv")
    q.add_argument("session")
    q.add_argument("--parquet", action="store_true")
    q.set_defaults(fn=cmd_csv)
    q = sub.add_parser("align")
    q.add_argument("session")
    q.add_argument("--pts")
    q.add_argument("--serial")
    q.add_argument("--date")
    q.set_defaults(fn=cmd_align)
    q = sub.add_parser("plot")
    q.add_argument("session")
    q.add_argument("--joints")
    q.add_argument("--pts")
    q.add_argument("--serial")
    q.add_argument("--date")
    q.add_argument("--t0", help="UTC HH:MM:SS[.f] window start")
    q.add_argument("--t1", help="UTC HH:MM:SS[.f] window end")
    q.add_argument("--out")
    q.set_defaults(fn=cmd_plot)
    a = p.parse_args(argv)
    return a.fn(a)


if __name__ == "__main__":
    sys.exit(main())
