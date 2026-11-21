#!/usr/bin/env python3
"""Flight recorder collector: listen on UDP, write one session file per run.

    tools/flight_collector.py                       # port 4220, hw_sessions/<today>/
    tools/flight_collector.py --label walk2         # ... flight_<UTC>_walk2.frec
    tools/flight_collector.py --forward laptop:4221 # ... and re-send each datagram there
    tools/flight_session.sh start [label] | stop | status   # the background form

The robot (firmware/main/flight_rec.h, `rec <mira-ip>`) sends one datagram per
control tick plus events and a 1 Hz hello. This writes every datagram, raw and
unparsed, with the host's receive time, into
hw_sessions/<local date>/flight_<UTC start>[_label].frec, and when the session
closes a <stem>.summary.json beside it: packets by type, the sequence holes
(every one a lost record), how many of them the robot itself admits to (ring
full / no WiFi) versus the network, per-source tick rates, and the time skew
between the robot's NTP clock and this host's.

A session ends (and the next packet opens a new file) when the collector
stops (SIGINT/SIGTERM), on SIGHUP, when the robot reboots (a new boot_id), or
after --split-gap seconds of silence. Files are opened lazily: a collector
that hears nothing writes nothing.

File format (.frec): b"FRECSESS1\\n", one line of JSON metadata, then records
of <u16 len><u64 host receive time, UTC us><packet>. tools/flight_reader.py
reads it. Stdlib only -- this runs as a long-lived service.

--forward HOST:PORT (repeatable) re-sends every datagram, byte for byte and in
arrival order, to each destination right after writing it: the live feed for
an operator console. Best effort -- one non-blocking send socket, a failed
send is counted (status line, the summary's "forward" block) and never
delays the file. Destinations are resolved once at start. Nothing is sent to
the robot: a destination at the address a datagram came from is skipped.

The files stay on mira: there is no upload anywhere in this file.
"""
from __future__ import annotations

import argparse
import datetime as dt
import json
import os
import signal
import socket
import statistics
import struct
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import flightrec as F  # noqa: E402

FILE_MAGIC = b"FRECSESS1\n"
REC = struct.Struct("<HQ")
MAX_GAPS_LISTED = 200


def repo_root() -> str:
    """The MAIN checkout (hw_sessions lives there, not in a worktree)."""
    here = os.path.dirname(HERE)
    try:
        common = subprocess.run(
            ["git", "-C", here, "rev-parse", "--path-format=absolute",
             "--git-common-dir"], capture_output=True, text=True, timeout=5,
            check=True).stdout.strip()
        return os.path.dirname(common)
    except (OSError, subprocess.SubprocessError):
        return here


def default_out_dir() -> str:
    return os.path.join(repo_root(), "hw_sessions", dt.date.today().isoformat())


def utc_stamp(us: int) -> str:
    return dt.datetime.fromtimestamp(us / 1e6, dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ")


def iso(us: int) -> str:
    return dt.datetime.fromtimestamp(us / 1e6, dt.timezone.utc).isoformat(timespec="milliseconds")


class SeqTracker:
    """Holes, duplicates and late arrivals in one boot's sequence space."""

    def __init__(self):
        self.first = None
        self.max = None
        self.missing: set[int] = set()
        self.received = 0
        self.duplicates = 0
        self.late = 0
        self._seen_recent: set[int] = set()

    def add(self, seq: int) -> None:
        self.received += 1
        if self.first is None:
            self.first = self.max = seq
            self._seen_recent.add(seq)
            return
        if seq > self.max:
            self.missing.update(range(self.max + 1, seq))
            self.max = seq
        elif seq in self.missing:
            self.missing.discard(seq)
            self.late += 1
        elif seq in self._seen_recent or seq >= self.first:
            self.duplicates += 1
            self.received -= 1
            return
        else:
            # older than anything seen: the session started mid-reorder
            self.missing.update(range(seq + 1, self.first))
            self.first = seq
        self._seen_recent.add(seq)
        if len(self._seen_recent) > 4096:
            floor = self.max - 2048
            self._seen_recent = {s for s in self._seen_recent if s >= floor}

    def gaps(self) -> list[list[int]]:
        """Missing seqs as [first, last] ranges."""
        out: list[list[int]] = []
        for s in sorted(self.missing):
            if out and s == out[-1][1] + 1:
                out[-1][1] = s
            else:
                out.append([s, s])
        return out


class Forwarder:
    """--forward: every datagram, unchanged, to each destination, best-effort.

    Its own non-blocking socket, sendto only: a full buffer, a refused port or
    an unreachable host costs one counted error and never a wait. `dests` is
    [(name, (ipv4, port))], resolved by the caller once (`resolve`).
    """

    def __init__(self, dests, sock=None):
        self.dests = list(dests)
        if sock is None:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setblocking(False)
        self.sock = sock
        self.sent = {n: 0 for n, _ in self.dests}
        self.errors = {n: 0 for n, _ in self.dests}
        self.skipped = {n: 0 for n, _ in self.dests}    # destination == the sender
        self.last_error: dict[str, str] = {}

    @staticmethod
    def resolve(specs) -> list:
        """[(host, port)] -> [(name, (ipv4, port))]; OSError names the bad one."""
        out = []
        for host, port in specs:
            try:
                out.append((f"{host}:{port}", F.resolve_ipv4(host, port)))
            except OSError as e:
                raise OSError(f"--forward {host}:{port}: cannot resolve ({e})") from e
        return out

    def send(self, data: bytes, src_ip: str | None = None) -> None:
        for name, addr in self.dests:
            # Never toward the robot. Loopback is a test or a replay (the rule
            # serve() uses for the robot's address too).
            if src_ip and addr[0] == src_ip and not src_ip.startswith("127."):
                self.skipped[name] += 1
                continue
            try:
                self.sock.sendto(data, addr)
                self.sent[name] += 1
            except OSError as e:
                self.errors[name] += 1
                self.last_error[name] = f"{type(e).__name__}: {e}"

    def counts(self) -> dict:
        return {n: (self.sent[n], self.errors[n], self.skipped[n]) for n, _ in self.dests}

    def since(self, before: dict) -> dict:
        """Per destination, the counts accumulated since `before` (counts())."""
        out = {}
        for name, addr in self.dests:
            s0, e0, k0 = before.get(name, (0, 0, 0))
            out[name] = dict(addr=f"{addr[0]}:{addr[1]}", sent=self.sent[name] - s0,
                             errors=self.errors[name] - e0,
                             skipped_sender=self.skipped[name] - k0,
                             last_error=self.last_error.get(name))
        return out

    def status(self) -> str:
        return "; ".join(f"fwd {n} {self.sent[n]} sent {self.errors[n]} err"
                         + (f" ({self.last_error[n]})" if n in self.last_error else "")
                         + (f" {self.skipped[n]} skipped" if self.skipped[n] else "")
                         for n, _ in self.dests)

    def close(self) -> None:
        self.sock.close()


class Session:
    """One open .frec file and its statistics."""

    def __init__(self, out_dir: str | None, label: str | None, first_rx_us: int, port: int,
                 forward: Forwarder | None = None):
        os.makedirs(out_dir, exist_ok=True)
        stem = "flight_" + utc_stamp(first_rx_us) + (f"_{label}" if label else "")
        path = os.path.join(out_dir, stem + ".frec")
        n = 1
        while os.path.exists(path):          # two sessions in one second
            n += 1
            path = os.path.join(out_dir, f"{stem}-{n}.frec")
        self.path = path
        self.start_us = first_rx_us
        self.last_rx_us = first_rx_us
        self.f = open(path, "wb")
        meta = dict(created_utc=iso(first_rx_us), host=socket.gethostname(),
                    port=port, label=label, format=1, decoder="tools/flightrec.py",
                    wire_version=F.VERSION)
        self.f.write(FILE_MAGIC + json.dumps(meta).encode() + b"\n")
        self.packets = 0
        self.bytes = 0
        self.bad = 0
        self.by_type = {"tick": 0, "event": 0, "hello": 0}
        self.by_source: dict[str, list[int]] = {}      # name -> t_boot_us list
        self.seq: dict[int, SeqTracker] = {}
        self.drops: dict[int, list[int]] = {}           # boot -> [ring0, net0, ring1, net1]
        self.skew_ms: list[float] = []                  # host rx - robot utc
        self.unsynced = 0
        self.hello: dict | None = None
        self.boot_id: int | None = None
        self.forward = forward
        self._fwd0 = forward.counts() if forward else None

    def write(self, data: bytes, rx_us: int, pkt: dict | None) -> None:
        self.f.write(REC.pack(len(data), rx_us) + data)
        self.packets += 1
        self.bytes += len(data)
        self.last_rx_us = rx_us
        if pkt is None:
            self.bad += 1
            return
        boot = pkt["boot_id"]
        self.boot_id = boot
        self.seq.setdefault(boot, SeqTracker()).add(pkt["seq"])
        d = self.drops.setdefault(boot, [pkt["drop_ring"], pkt["drop_net"]] * 2)
        d[2] = max(d[2], pkt["drop_ring"])
        d[3] = max(d[3], pkt["drop_net"])
        if pkt["t_utc_us"]:
            self.skew_ms.append((rx_us - pkt["t_utc_us"]) / 1000.0)
        else:
            self.unsynced += 1
        t = pkt["type"]
        if t == F.T_TICK:
            self.by_type["tick"] += 1
            name = F.SOURCE_NAMES.get(pkt["source"], str(pkt["source"]))
            self.by_source.setdefault(name, []).append(pkt["t_boot_us"])
        elif t == F.T_EVENT:
            self.by_type["event"] += 1
        elif t == F.T_HELLO:
            self.by_type["hello"] += 1
            self.hello = {k: v for k, v in pkt.items()}

    def summary(self) -> dict:
        boots = {}
        for boot, tr in self.seq.items():
            d = self.drops.get(boot, [0, 0, 0, 0])
            missing = len(tr.missing)
            robot_dropped = (d[2] - d[0]) + (d[3] - d[1])
            gaps = tr.gaps()
            boots[f"{boot:08x}"] = dict(
                seq_first=tr.first, seq_last=tr.max, received=tr.received,
                missing=missing, gaps=gaps[:MAX_GAPS_LISTED],
                gaps_total=len(gaps), duplicates=tr.duplicates, late=tr.late,
                robot_drop_ring=d[2] - d[0], robot_drop_net=d[3] - d[1],
                lost_on_network=max(0, missing - robot_dropped))
        rates = {}
        for name, ts in self.by_source.items():
            ts = sorted(ts)
            dts = [(b - a) / 1e6 for a, b in zip(ts, ts[1:]) if 0 < b - a < 2e6]
            rates[name] = dict(
                ticks=len(ts),
                median_hz=round(1.0 / statistics.median(dts), 2) if dts else None,
                p95_period_ms=round(sorted(dts)[int(0.95 * (len(dts) - 1))] * 1e3, 2) if dts else None)
        skew = sorted(self.skew_ms)
        fwd = dict(forward=self.forward.since(self._fwd0)) if self.forward else {}
        return dict(
            file=os.path.basename(self.path),
            start_utc=iso(self.start_us), end_utc=iso(self.last_rx_us),
            duration_s=round((self.last_rx_us - self.start_us) / 1e6, 3),
            packets=self.packets, bytes=self.bytes, undecodable=self.bad,
            by_type=self.by_type, boots=boots, tick_rates=rates,
            robot_minus_host_clock=dict(
                note="host receive time minus robot NTP stamp: network latency "
                     "+ the two clocks' offset",
                median_ms=round(statistics.median(skew), 2) if skew else None,
                min_ms=round(skew[0], 2) if skew else None,
                p95_ms=round(skew[int(0.95 * (len(skew) - 1))], 2) if skew else None),
            packets_without_utc=self.unsynced,
            firmware=(self.hello or {}).get("fw"),
            last_hello={k: v for k, v in (self.hello or {}).items()
                        if k not in ("type",)},
            **fwd,
        )

    def close(self) -> dict:
        self.f.close()
        s = self.summary()
        with open(self.path[:-5] + ".summary.json", "w") as f:
            json.dump(s, f, indent=1)
        return s


class Collector:
    """The service logic, socket-free so the tests drive it directly."""

    def __init__(self, out_dir: str | None, label: str | None = None, split_gap_s: float = 30.0,
                 split_on_boot: bool = True, port: int = F.DEFAULT_PORT, log=print,
                 forward: Forwarder | None = None):
        self.out_dir = out_dir
        self.label = label
        self.split_gap_us = int(split_gap_s * 1e6)
        self.split_on_boot = split_on_boot
        self.port = port
        self.log = log
        self.forward = forward
        self.session: Session | None = None
        self.closed: list[dict] = []

    def handle(self, data: bytes, rx_us: int, src_ip: str | None = None) -> None:
        try:
            pkt = F.decode(data)
        except F.DecodeError:
            pkt = None
        s = self.session
        if s is not None:
            if rx_us - s.last_rx_us > self.split_gap_us:
                self.rotate(f"{(rx_us - s.last_rx_us) / 1e6:.0f} s of silence")
            elif (self.split_on_boot and pkt is not None and s.boot_id is not None
                  and pkt["boot_id"] != s.boot_id):
                self.rotate(f"robot rebooted (boot {s.boot_id:08x} -> {pkt['boot_id']:08x})")
        if self.session is None:
            # no --out: today's directory, decided per session (a service
            # that runs for days still files each run under its own date)
            out = self.out_dir or default_out_dir()
            self.session = Session(out, self.label, rx_us, self.port, self.forward)
            self.log(f"session open: {self.session.path}")
        self.session.write(data, rx_us, pkt)
        if self.forward is not None:
            self.forward.send(data, src_ip)

    def rotate(self, why: str = "rotate") -> dict | None:
        s = self.session
        if s is None:
            return None
        self.session = None
        summ = s.close()
        self.closed.append(summ)
        lost = sum(b["missing"] for b in summ["boots"].values())
        fwd = "".join(f", fwd {n} {f['sent']} sent {f['errors']} err"
                      for n, f in summ.get("forward", {}).items())
        self.log(f"session closed ({why}): {s.path}  {summ['packets']} packets, "
                 f"{summ['duration_s']} s, {lost} missing{fwd}")
        return summ

    def status(self) -> str:
        s = self.session
        fwd = f"; {self.forward.status()}" if self.forward is not None else ""
        if s is None:
            return "idle (no open session)" + fwd
        lost = sum(len(t.missing) for t in s.seq.values())
        return (f"{os.path.basename(s.path)}: {s.packets} pkts "
                f"(tick {s.by_type['tick']}, event {s.by_type['event']}, "
                f"hello {s.by_type['hello']}), {lost} missing, {s.bad} bad" + fwd)


def now_us() -> int:
    return time.time_ns() // 1000


def note_robot_addr(ip: str) -> None:
    """Remember where the robot is: hw_sessions/robot_addr in the main checkout.

    tools/estop.py reads it, so the operator's E-stop needs no --host while a
    collector is running. Written only when the address changes, atomically.
    """
    d = os.path.join(repo_root(), "hw_sessions")
    try:
        os.makedirs(d, exist_ok=True)
        tmp = os.path.join(d, ".robot_addr.tmp")
        with open(tmp, "w") as f:
            f.write(ip + "\n")
        os.replace(tmp, os.path.join(d, "robot_addr"))
    except OSError as e:
        print(f"note: could not record the robot address: {e}", flush=True)


def serve(args) -> int:
    forward = None
    if args.forward:
        try:
            forward = Forwarder(Forwarder.resolve(args.forward))
        except OSError as e:
            print(f"flight collector: {e}", file=sys.stderr, flush=True)
            return 2
    col = Collector(args.out, args.label, args.split_gap, not args.no_split_on_boot,
                    args.port, log=lambda m: print(m, flush=True), forward=forward)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 << 20)
    sock.bind((args.bind, args.port))
    sock.settimeout(0.5)
    stop = {"now": False, "rotate": False}

    def on_stop(*_):
        stop["now"] = True

    def on_hup(*_):
        stop["rotate"] = True

    signal.signal(signal.SIGINT, on_stop)
    signal.signal(signal.SIGTERM, on_stop)
    signal.signal(signal.SIGHUP, on_hup)
    print(f"flight collector: udp {args.bind}:{args.port} -> "
          f"{args.out or default_out_dir() + ' (per-session date)'}"
          + ("".join(f", forward -> {n} ({a[0]}:{a[1]})" for n, a in forward.dests)
             if forward else ""), flush=True)
    t_end = time.monotonic() + args.duration if args.duration else None
    next_status = time.monotonic() + args.status_every
    robot_ip = None
    while not stop["now"]:
        if t_end is not None and time.monotonic() >= t_end:
            break
        if stop["rotate"]:
            stop["rotate"] = False
            col.rotate("SIGHUP")
        try:
            data, src = sock.recvfrom(2048)
        except socket.timeout:
            data = None
        if data:
            col.handle(data, now_us(), src[0])
            # Loopback is a test or a replay, never the robot.
            if src[0] != robot_ip and not src[0].startswith("127."):
                robot_ip = src[0]
                note_robot_addr(robot_ip)
        if args.status_every and time.monotonic() >= next_status:
            next_status = time.monotonic() + args.status_every
            print("status: " + col.status(), flush=True)
    col.rotate("collector stopped")
    sock.close()
    if forward is not None:
        print("forward totals: " + forward.status(), flush=True)
        forward.close()
    return 0


def forward_spec(s: str) -> tuple[str, int]:
    try:
        return F.parse_hostport(s)
    except ValueError as e:
        raise argparse.ArgumentTypeError(str(e)) from e


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("--port", type=int, default=F.DEFAULT_PORT)
    p.add_argument("--bind", default="0.0.0.0")
    p.add_argument("--out", default=None, help="session directory "
                   "(default: <main checkout>/hw_sessions/<local date>)")
    p.add_argument("--label", default=None, help="appended to the file name")
    p.add_argument("--split-gap", type=float, default=30.0,
                   help="seconds of silence that end a session (default 30)")
    p.add_argument("--no-split-on-boot", action="store_true",
                   help="keep one file across a robot reboot")
    p.add_argument("--duration", type=float, default=0.0, help="stop after N s (0 = run)")
    p.add_argument("--status-every", type=float, default=10.0,
                   help="print a status line every N s (0 = never)")
    p.add_argument("--forward", action="append", default=[], type=forward_spec,
                   metavar="HOST:PORT",
                   help="after writing each datagram, re-send it unchanged here "
                        "(repeatable; best-effort, send errors are counted)")
    args = p.parse_args(argv)
    return serve(args)


if __name__ == "__main__":
    sys.exit(main())
