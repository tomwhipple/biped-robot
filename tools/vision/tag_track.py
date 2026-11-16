#!/usr/bin/env python3
"""Track the robot on the board from AprilTags (36h11): position, heading, tilt, edge margins.

    tools/vision/tag_track.py VIDEO.mp4 [--pts VIDEO.pts] [-o track.jsonl|.csv]
        [--annotate out.mp4] [--config vision.json] [--stride 1] [--start S] [--end S]
    tools/vision/tag_track.py IMAGE.jpg --calibrate        # report f, board residual, camera position

Per frame: detect tags; the BOARD tags (ids 0-5, known positions in the config)
give the camera pose by planar PnP; the focal length is estimated from them
(square pixels, principal point at the image centre) unless the config sets it;
the ROBOT torso tags (10 front, 11 back, 12 top; mounts in the config) give the
body pose in the board frame -> x/y (mm from the board origin), heading, pitch,
roll, and the distance to each board edge. Feet (13/14) are reported separately.
Tags hidden: the last pose is held and flagged stale (stale_s), dropped after
lost_s. Frames are keyed by wall-clock UTC from the .pts file (else --t0 + n/fps).
The camera is assumed fixed: the board pose is the median over the first frames
that show >= 2 board tags; board_rms_px per frame says when that stopped being true.
Conventions: tools/vision/vgeom.py docstring. All lengths in the config are
measured values (default = nominal): see README "What to measure".
"""
import argparse
import csv
import json
import os
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import vgeom as G  # noqa: E402
from tagdefs import detect, make_detector  # noqa: E402


def _angle(R1, R2):
    return float(np.degrees(np.arccos(np.clip((np.trace(R1.T @ R2) - 1) / 2, -1, 1))))


class Tracker:
    def __init__(self, cfg=None, size=None):
        self.cfg = cfg or G.load_config()
        self.size = tuple(size or self.cfg["image_size"])
        self.det = make_detector()
        self.K = None
        self.f_info = None
        self.pose = None                  # (rvec, tvec) board -> camera
        self.R_cb = self.t_cb = None
        self._last = None                 # last good body pose dict
        self._last_t = None
        self._fresh = False               # a held pose exists that is recent enough to trust

    # -- calibration ------------------------------------------------------------
    def set_camera(self, f):
        c = self.cfg["camera"]
        self.K = G.make_K(f, self.size, c.get("cx"), c.get("cy"))

    def set_board_pose(self, rvec, tvec):
        self.pose = (rvec, tvec)
        self.R_cb = cv2.Rodrigues(rvec)[0]
        self.t_cb = tvec.reshape(3)

    def calibrate(self, frames_dets, f=None):
        """frames_dets: detection dicts of frames showing the board. Sets K and the board
        pose; returns a report dict (None if no frame shows >= 2 board tags)."""
        good = [d for d in frames_dets if len([i for i in d if str(i) in self.cfg["board"]["tags"]]) >= 2]
        if not good:
            return None
        f = f or self.cfg["camera"].get("f_px")
        info = None
        if f is None:
            info = G.estimate_focal(self.cfg, good, self.size)
            f = info["f"]
        self.set_camera(f)
        self.f_info = info
        poses = []
        for d in good:
            obj, img, ids = G.stack_board(self.cfg, d)
            r = G.solve_board_pose(obj, img, self.K)
            if r:
                poses.append((r[0].reshape(3), r[1].reshape(3), r[2], len(ids)))
        Rs = [cv2.Rodrigues(p[0])[0] for p in poses]
        R = G.rot_mean(Rs)
        t = np.median([p[1] for p in poses], axis=0)
        self.set_board_pose(cv2.Rodrigues(R)[0], t.reshape(3, 1))
        rms = [G.reproj_rms(*self._board_pts(d), self.pose[0], self.pose[1], self.K) for d in good]
        return dict(f_px=float(f), f_estimate=info, board_frames=len(good),
                    board_rms_px_median=float(np.median(rms)), board_rms_px_max=float(np.max(rms)),
                    camera_position_board_mm=(-self.R_cb.T @ self.t_cb).round(1).tolist(),
                    tags_seen=sorted({i for d in good for i in d if str(i) in self.cfg["board"]["tags"]}))

    def _board_pts(self, d):
        obj, img, _ = G.stack_board(self.cfg, d)
        return obj, img

    # -- per frame --------------------------------------------------------------
    def _tag_poses(self, corners, size):
        obj = G.tag_corners_local(size)
        n, rv, tv, err = cv2.solvePnPGeneric(obj, corners.astype(np.float64), self.K, None,
                                             flags=cv2.SOLVEPNP_IPPE_SQUARE)
        out = []
        for i in range(n):
            R_ct = cv2.Rodrigues(rv[i])[0]
            out.append((R_ct, tv[i].reshape(3), float(np.ravel(err)[i])))
        return out

    def _to_board(self, R_ct, t_ct):
        return self.R_cb.T @ R_ct, self.R_cb.T @ (t_ct - self.t_cb)

    def process(self, gray, t=None, dets=None):
        """-> result dict for one frame (see module doc)."""
        dets = detect(gray, self.det) if dets is None else dets
        cfg = self.cfg
        res = dict(t=t, tags=sorted(dets))
        board_ids = [i for i in dets if str(i) in cfg["board"]["tags"]]
        res["n_board_tags"] = len(board_ids)
        if self.K is None or self.pose is None:
            res["error"] = "not calibrated"
            return res
        if board_ids:
            obj, img, _ = G.stack_board(cfg, dets)
            res["board_rms_px"] = G.reproj_rms(obj, img, self.pose[0], self.pose[1], self.K)
        # robot torso tags -> candidate body poses
        cands = []
        for i in sorted(dets):
            e = cfg["robot"]["tags"].get(str(i))
            if e is None:
                continue
            px = G.mean_edge_px(dets[i])
            if px < cfg["min_tag_px"]:
                continue
            Rm, tm = G.robot_tag_pose(e)
            opts = []
            for R_ct, t_ct, err in self._tag_poses(dets[i], e["size"]):
                if err > cfg["max_tag_rms_px"]:
                    continue
                Rbt, tbt = self._to_board(R_ct, t_ct)
                Rb = Rbt @ Rm.T
                tb = tbt - Rb @ tm
                if Rb[2, 2] < 0.2:              # robot is not upside down / on its side
                    continue
                opts.append((Rb, tb, err))
            if opts:
                cands.append(dict(id=i, px=px, opts=opts, corners=dets[i]))
        self._fresh = (self._last is not None and t is not None and self._last_t is not None
                       and t - self._last_t <= self.cfg["hold_s"] * 2)
        body = self._fuse(cands)
        pose_out = None
        if body is not None:
            Rb, tb, used, sigma = body
            hd, pt, rl = G.euler_from_board_R(Rb)
            pose_out = dict(x_mm=float(tb[0]), y_mm=float(tb[1]), z_mm=float(tb[2]), heading_deg=float(hd),
                            pitch_deg=float(pt), roll_deg=float(rl * cfg["roll_sign"]),
                            used_tags=used, **sigma)
            self._last, self._last_t = pose_out, t
            res.update(pose_out, stale=False, stale_s=0.0)
        elif self._last is not None and t is not None and self._last_t is not None:
            age = t - self._last_t
            if age <= cfg["lost_s"]:
                res.update({k: v for k, v in self._last.items()}, stale=True, stale_s=float(age))
            else:
                res.update(stale=None, stale_s=float(age))
        else:
            res.update(stale=None)
        if res.get("x_mm") is not None:
            m = G.edge_margins(cfg, res["x_mm"], res["y_mm"])
            res.update({f"margin_{k}_mm": float(v) for k, v in m.items()})
            k = min(m, key=m.get)
            res.update(margin_min_mm=float(m[k]), margin_min_edge=k)
        # feet
        for i, fe in cfg["robot"].get("foot_tags", {}).items():
            i = int(i)
            if i in dets and G.mean_edge_px(dets[i]) >= cfg["min_tag_px"]:
                best = None
                for R_ct, t_ct, err in self._tag_poses(dets[i], fe["size"]):
                    Rbt, tbt = self._to_board(R_ct, t_ct)
                    if best is None or Rbt[2, 2] > best[0][2, 2]:
                        best = (Rbt, tbt)
                up = best[0][:, 1]
                res[f"foot_{fe['side']}"] = dict(x_mm=float(best[1][0]), y_mm=float(best[1][1]),
                                                 heading_deg=float(np.degrees(np.arctan2(-up[0], up[1]))))
        return res

    def _fuse(self, cands):
        """Joint fit of the body pose to the corners of every visible torso tag (MAP: a weak
        'roughly upright' prior on pitch/roll settles the two-fold planar-PnP ambiguity of a
        single small tag). Starts from each combination of per-tag PnP solutions, keeps the
        cheapest. -> (R_board_body, t_board_body, used ids, posterior sigma dict) or None."""
        if not cands:
            return None
        import itertools
        from scipy.optimize import least_squares
        cfg = self.cfg
        prior = cfg.get("tilt_prior_deg", 15.0)
        sig_px = cfg.get("corner_sigma_px", 0.5)
        tags = []
        for c in cands:
            e = cfg["robot"]["tags"][str(c["id"])]
            tags.append((G.tag_corners_local(e["size"]), *G.robot_tag_pose(e), c["corners"]))

        def model(p):
            Rb = G.board_R_from_euler(p[3], p[4], p[5])
            res = []
            for obj, Rm, tm, img in tags:
                P = (obj @ (Rb @ Rm).T) + Rb @ tm + p[:3]
                pc = P @ self.R_cb.T + self.t_cb
                uv = pc @ self.K.T
                res.append(((uv[:, :2] / uv[:, 2:3]) - img).ravel() / sig_px)
            return np.concatenate(res)

        def resid(p):
            r = model(p)
            return np.concatenate([r, [p[4] / prior, p[5] / prior]]) if prior else r
        best, sols = None, []
        for combo in itertools.product(*[range(len(c["opts"])) for c in cands]):
            sel = [c["opts"][k] for c, k in zip(cands, combo)]
            R = G.rot_mean([s[0] for s in sel])
            t0 = np.mean([s[1] for s in sel], axis=0)
            hd, pt, rl = G.euler_from_board_R(R)
            sol = least_squares(resid, [*t0, hd, pt, rl], method="lm", x_scale=[10, 10, 10, 2, 2, 2])
            sols.append(sol)
            if best is None or sol.cost < best.cost:
                best = sol
        near = [s for s in sols if 2 * (s.cost - best.cost) < 9.0]       # fit about as well as the best
        resolved = False
        if len(near) > 1 and self._fresh:
            Rl = G.board_R_from_euler(self._last["heading_deg"], self._last["pitch_deg"],
                                      self._last["roll_deg"] * cfg["roll_sign"])
            cont = min(near, key=lambda s: _angle(G.board_R_from_euler(*s.x[3:]), Rl))
            resolved = cont is not best and _angle(G.board_R_from_euler(*cont.x[3:]),
                                                   G.board_R_from_euler(*best.x[3:])) > 10
            best = cont
        p = best.x
        # a clearly different second solution that fits almost as well = ambiguous pose
        Rbest = G.board_R_from_euler(*p[3:])
        alts = [s for s in sols if _angle(G.board_R_from_euler(*s.x[3:]), Rbest) > 10]
        d_chi2 = float(2 * (min(s.cost for s in alts) - best.cost)) if alts else float("inf")
        Rb = G.board_R_from_euler(p[3], p[4], p[5])
        # data-only uncertainty of the angles (what the corners alone say)
        J = best.jac[:-2] if prior else best.jac
        try:
            cov = np.linalg.pinv(J.T @ J)
            sd = np.sqrt(np.abs(np.diag(cov)))
            sigma = dict(heading_sigma_deg=float(sd[3]), pitch_sigma_deg=float(sd[4]), roll_sigma_deg=float(sd[5]))
        except np.linalg.LinAlgError:
            sigma = {}
        sigma["alt_dchi2"] = None if np.isinf(d_chi2) else d_chi2
        sigma["ambiguous"] = bool(d_chi2 < 9.0)
        sigma["ambiguity_resolved_by_continuity"] = resolved
        return Rb, p[:3], [c["id"] for c in cands], sigma

    # -- drawing -------------------------------------------------------------------
    def annotate(self, bgr, res, dets):
        img = bgr.copy()
        for i, c in dets.items():
            cv2.polylines(img, [c.astype(np.int32).reshape(-1, 1, 2)], True, (0, 255, 0), 1)
            cv2.putText(img, str(i), tuple(c[0].astype(int)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        if self.pose is not None:
            W, L = self.cfg["board"]["width_mm"], self.cfg["board"]["length_mm"]
            P = np.array([[0, 0, 0], [W, 0, 0], [W, L, 0], [0, L, 0.0]])
            front = (self.R_cb @ P.T).T[:, 2] > 10
            if front.all():
                p, _ = cv2.projectPoints(P.reshape(-1, 1, 3), *self.pose, self.K, None)
                cv2.polylines(img, [p.astype(np.int32)], True, (255, 128, 0), 1)
        if res.get("x_mm") is not None:
            col = (0, 0, 255) if res.get("stale") else (0, 255, 0)
            txt = (f"x {res['x_mm']:.0f} y {res['y_mm']:.0f} hd {res['heading_deg']:.0f} "
                   f"pit {res['pitch_deg']:.0f} rol {res['roll_deg']:.0f} edge {res['margin_min_mm']:.0f}"
                   f"{' STALE' if res.get('stale') else ''}")
            cv2.putText(img, txt, (8, 660), cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2)
        return img


def read_pts(path):
    ts = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line and not line.startswith("#"):
                ts.append(float(line) / 1000.0)
    return ts


def frames(path, pts=None, stride=1, start=None, end=None, t0=0.0):
    """Yield (index, t_utc, bgr) from a video; t from the .pts, else t0 + n/fps."""
    cap = cv2.VideoCapture(path)
    fps = cap.get(cv2.CAP_PROP_FPS) or 25.0
    if pts is None and os.path.exists(os.path.splitext(path)[0] + ".pts"):
        pts = read_pts(os.path.splitext(path)[0] + ".pts")
    elif isinstance(pts, str):
        pts = read_pts(pts)
    n = -1
    while True:
        ok, bgr = cap.read()
        if not ok:
            break
        n += 1
        t = pts[n] if pts is not None and n < len(pts) else t0 + n / fps
        if start is not None and t < start:
            continue
        if end is not None and t > end:
            break
        if n % stride:
            continue
        yield n, t, bgr
    cap.release()


def track_video(path, cfg=None, pts=None, stride=1, start=None, end=None, calib_frames=60,
                annotate=None, t0=0.0, progress=False):
    """Run the tracker over a video -> (rows, calibration report)."""
    tr = Tracker(cfg)
    det_cache, rows = [], []
    for n, t, bgr in frames(path, pts, 1, start, end, t0):
        if n % 5 == 0 and len(det_cache) < calib_frames:
            d = detect(cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY), tr.det)
            if len([i for i in d if str(i) in tr.cfg["board"]["tags"]]) >= 2:
                det_cache.append(d)
        if len(det_cache) >= calib_frames or n > 1500:
            break
    cal = tr.calibrate(det_cache)
    if cal is None:
        return [], None
    writer = None
    for n, t, bgr in frames(path, pts, stride, start, end, t0):
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        d = detect(gray, tr.det)
        r = tr.process(gray, t, d)
        r["frame"] = n
        rows.append(r)
        if annotate:
            out = tr.annotate(bgr, r, d)
            if writer is None:
                writer = cv2.VideoWriter(annotate, cv2.VideoWriter_fourcc(*"mp4v"), 25.0 / max(stride, 1),
                                         (out.shape[1], out.shape[0]))
            writer.write(out)
    if writer:
        writer.release()
    return rows, cal


def write_rows(rows, path):
    if path.endswith(".csv"):
        keys = []
        for r in rows:
            for k in r:
                if k not in keys:
                    keys.append(k)
        with open(path, "w", newline="") as f:
            w = csv.DictWriter(f, keys)
            w.writeheader()
            for r in rows:
                w.writerow({k: (json.dumps(v) if isinstance(v, (list, dict)) else v) for k, v in r.items()})
    else:
        with open(path, "w") as f:
            for r in rows:
                f.write(json.dumps(r) + "\n")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("input")
    ap.add_argument("--pts")
    ap.add_argument("-o", "--out")
    ap.add_argument("--annotate")
    ap.add_argument("--config")
    ap.add_argument("--stride", type=int, default=1)
    ap.add_argument("--start", type=float, help="UTC epoch s")
    ap.add_argument("--end", type=float)
    ap.add_argument("--calibrate", action="store_true", help="only report f, board pose residuals, camera position")
    a = ap.parse_args(argv)
    cfg = G.load_config(a.config)
    if a.input.lower().endswith((".jpg", ".jpeg", ".png")):
        gray = cv2.imread(a.input, cv2.IMREAD_GRAYSCALE)
        tr = Tracker(cfg, (gray.shape[1], gray.shape[0]))
        d = detect(gray, tr.det)
        cal = tr.calibrate([d])
        print(json.dumps(cal, indent=1))
        if not a.calibrate and cal:
            print(json.dumps(tr.process(gray, None, d), indent=1))
        return 0 if cal else 2
    if a.calibrate:
        tr = Tracker(cfg)
        ds = []
        for n, t, bgr in frames(a.input, a.pts, 5, a.start, a.end):
            ds.append(detect(cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY), tr.det))
            if len(ds) >= 60:
                break
        cal = tr.calibrate(ds)
        print(json.dumps(cal, indent=1))
        return 0 if cal else 2
    rows, cal = track_video(a.input, cfg, a.pts, a.stride, a.start, a.end, annotate=a.annotate)
    if cal is None:
        print("tag_track: no frame shows two board tags; nothing to track", file=sys.stderr)
        return 2
    print(json.dumps(cal), file=sys.stderr)
    if a.out:
        write_rows(rows, a.out)
    else:
        for r in rows:
            print(json.dumps(r))
    return 0


if __name__ == "__main__":
    sys.exit(main())
