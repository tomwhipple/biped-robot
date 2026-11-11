# Lateral shift ladder as sent 2026-10-25 (both-feet-down poses only)

Source: hw_sessions/2026-10-25/latR_serial.log, latL_serial.log. R ladder = pelvis toward the robot's RIGHT foot. Pose strings verbatim (17 ticks + s<ms>).

Identification: s2000 moves are the shifts (7 per side, matching NOTES' 7 multipliers 0.5,0.75,1.0,1.1,1.2,1.3,1.4 x 62 mm); each is followed by an s1500 pose that differs from it only in R/L hip_pitch, knee, ankle (5 mm foot lift, EXCLUDED) and an s1500 pose byte-identical to the shift (lower, EXCLUDED). No `B<L|R>` token anywhere in either log (grep count 0): no balance loop. Sequence is stand s3000 -> crouch s3000 -> [shift s2000, lift, lower] x7 -> crouch s3000 -> stand s3000 -> home. Rungs are commanded rung-to-rung (shift k -> lift -> lower to shift k -> shift k+1), NOT returning to crouch between rungs.

Degrees: dir*dticks*360/4096 from one_leg.CAL; sign is the joint-positive direction, so just check magnitudes. Pitch chain = hip_pitch/knee/ankle.


## R ladder (latR_serial.log)

- stand (18:55:40.245Z): `pose 2082 1997 2125 2045 1991 2058 1977 2091 2184 2153 2108 1997 2048 2048 2048 2048 2048 s3000`
- crouch (18:55:43.847Z): `pose 2082 1717 2685 1765 1991 1778 2537 1811 2184 2153 2108 1997 2048 2048 2048 2048 2048 s3000`
  - from stand: R_hroll=+0.0 R_hpitch=-24.6 R_k=-49.2 R_a=-24.6 L_hroll=+0.0 L_hpitch=-24.6 L_k=-49.2 L_a=-24.6 R_ar=+0.0 L_ar=+0.0
- rung 1: x0.5 = 31.0 mm (18:56:31.453Z): `pose 2017 1739 2662 1766 1926 1800 2514 1812 2184 2153 2043 1932 2048 2048 2048 2048 2048 s2000`
  - from stand (deg): R_hroll=+5.7 R_hpitch=-22.7 R_k=-47.2 R_a=-24.5 L_hroll=+5.7 L_hpitch=-22.7 L_k=-47.2 L_a=-24.5 R_ar=-5.7 L_ar=-5.7
  - from crouch (deg): R_hroll=+5.7 R_hpitch=+1.9 R_k=+2.0 R_a=+0.1 L_hroll=+5.7 L_hpitch=+1.9 L_k=+2.0 L_a=+0.1 R_ar=-5.7 L_ar=-5.7
- rung 2: x0.75 = 46.5 mm (18:56:40.558Z): `pose 1984 1759 2632 1776 1893 1820 2484 1822 2184 2153 2010 1899 2048 2048 2048 2048 2048 s2000`
  - from stand (deg): R_hroll=+8.6 R_hpitch=-20.9 R_k=-44.6 R_a=-23.6 L_hroll=+8.6 L_hpitch=-20.9 L_k=-44.6 L_a=-23.6 R_ar=-8.6 L_ar=-8.6
  - from crouch (deg): R_hroll=+8.6 R_hpitch=+3.7 R_k=+4.7 R_a=+1.0 L_hroll=+8.6 L_hpitch=+3.7 L_k=+4.7 L_a=+1.0 R_ar=-8.6 L_ar=-8.6
- rung 3: x1.0 = 62.0 mm (18:56:49.660Z): `pose 1953 1786 2588 1793 1862 1847 2440 1839 2184 2153 1979 1868 2048 2048 2048 2048 2048 s2000`
  - from stand (deg): R_hroll=+11.3 R_hpitch=-18.5 R_k=-40.7 R_a=-22.1 L_hroll=+11.3 L_hpitch=-18.5 L_k=-40.7 L_a=-22.1 R_ar=-11.3 L_ar=-11.3
  - from crouch (deg): R_hroll=+11.3 R_hpitch=+6.1 R_k=+8.5 R_a=+2.5 L_hroll=+11.3 L_hpitch=+6.1 L_k=+8.5 L_a=+2.5 R_ar=-11.3 L_ar=-11.3
- rung 4: x1.1 = 68.2 mm (18:56:58.707Z): `pose 1940 1799 2565 1803 1849 1860 2417 1849 2184 2153 1966 1855 2048 2048 2048 2048 2048 s2000`
  - from stand (deg): R_hroll=+12.5 R_hpitch=-17.4 R_k=-38.7 R_a=-21.3 L_hroll=+12.5 L_hpitch=-17.4 L_k=-38.7 L_a=-21.3 R_ar=-12.5 L_ar=-12.5
  - from crouch (deg): R_hroll=+12.5 R_hpitch=+7.2 R_k=+10.6 R_a=+3.3 L_hroll=+12.5 L_hpitch=+7.2 L_k=+10.6 L_a=+3.3 R_ar=-12.5 L_ar=-12.5
- rung 5: x1.2 = 74.4 mm (18:57:07.799Z): `pose 1928 1814 2539 1814 1837 1875 2391 1860 2184 2153 1954 1843 2048 2048 2048 2048 2048 s2000`
  - from stand (deg): R_hroll=+13.5 R_hpitch=-16.1 R_k=-36.4 R_a=-20.3 L_hroll=+13.5 L_hpitch=-16.1 L_k=-36.4 L_a=-20.3 R_ar=-13.5 L_ar=-13.5
  - from crouch (deg): R_hroll=+13.5 R_hpitch=+8.5 R_k=+12.8 R_a=+4.3 L_hroll=+13.5 L_hpitch=+8.5 L_k=+12.8 L_a=+4.3 R_ar=-13.5 L_ar=-13.5
- rung 6: x1.3 = 80.6 mm (18:57:16.929Z): `pose 1915 1831 2509 1827 1824 1892 2361 1873 2184 2153 1941 1830 2048 2048 2048 2048 2048 s2000`
  - from stand (deg): R_hroll=+14.7 R_hpitch=-14.6 R_k=-33.8 R_a=-19.2 L_hroll=+14.7 L_hpitch=-14.6 L_k=-33.8 L_a=-19.2 R_ar=-14.7 L_ar=-14.7
  - from crouch (deg): R_hroll=+14.7 R_hpitch=+10.0 R_k=+15.5 R_a=+5.5 L_hroll=+14.7 L_hpitch=+10.0 L_k=+15.5 L_a=+5.5 R_ar=-14.7 L_ar=-14.7
- rung 7: x1.4 = 86.8 mm (18:57:26.036Z): `pose 1903 1850 2474 1843 1812 1911 2326 1889 2184 2153 1929 1818 2048 2048 2048 2048 2048 s2000`
  - from stand (deg): R_hroll=+15.7 R_hpitch=-12.9 R_k=-30.7 R_a=-17.8 L_hroll=+15.7 L_hpitch=-12.9 L_k=-30.7 L_a=-17.8 R_ar=-15.7 L_ar=-15.7
  - from crouch (deg): R_hroll=+15.7 R_hpitch=+11.7 R_k=+18.5 R_a=+6.9 L_hroll=+15.7 L_hpitch=+11.7 L_k=+18.5 L_a=+6.9 R_ar=-15.7 L_ar=-15.7
- excluded per rung: lift 5 mm + lower (see json `excluded`); tail: 18:58:02.133Z `2082 1717 2685 1765 1991 1778 ...`; 18:58:05.736Z `2082 1997 2125 2045 1991 2058 ...`

## L ladder (latL_serial.log)

- stand (19:04:58.442Z): `pose 2082 1997 2125 2045 1991 2058 1977 2091 2184 2153 2108 1997 2048 2048 2048 2048 2048 s3000`
- crouch (19:05:02.043Z): `pose 2082 1717 2685 1765 1991 1778 2537 1811 2184 2153 2108 1997 2048 2048 2048 2048 2048 s3000`
  - from stand: R_hroll=+0.0 R_hpitch=-24.6 R_k=-49.2 R_a=-24.6 L_hroll=+0.0 L_hpitch=-24.6 L_k=-49.2 L_a=-24.6 R_ar=+0.0 L_ar=+0.0
- rung 1: x0.5 = 31.0 mm (19:05:06.650Z): `pose 2147 1739 2662 1766 2056 1800 2514 1812 2184 2153 2173 2062 2048 2048 2048 2048 2048 s2000`
  - from stand (deg): R_hroll=-5.7 R_hpitch=-22.7 R_k=-47.2 R_a=-24.5 L_hroll=-5.7 L_hpitch=-22.7 L_k=-47.2 L_a=-24.5 R_ar=+5.7 L_ar=+5.7
  - from crouch (deg): R_hroll=-5.7 R_hpitch=+1.9 R_k=+2.0 R_a=+0.1 L_hroll=-5.7 L_hpitch=+1.9 L_k=+2.0 L_a=+0.1 R_ar=+5.7 L_ar=+5.7
- rung 2: x0.75 = 46.5 mm (19:05:15.694Z): `pose 2180 1759 2632 1776 2089 1820 2484 1822 2184 2153 2206 2095 2048 2048 2048 2048 2048 s2000`
  - from stand (deg): R_hroll=-8.6 R_hpitch=-20.9 R_k=-44.6 R_a=-23.6 L_hroll=-8.6 L_hpitch=-20.9 L_k=-44.6 L_a=-23.6 R_ar=+8.6 L_ar=+8.6
  - from crouch (deg): R_hroll=-8.6 R_hpitch=+3.7 R_k=+4.7 R_a=+1.0 L_hroll=-8.6 L_hpitch=+3.7 L_k=+4.7 L_a=+1.0 R_ar=+8.6 L_ar=+8.6
- rung 3: x1.0 = 62.0 mm (19:05:24.766Z): `pose 2211 1786 2588 1793 2120 1847 2440 1839 2184 2153 2237 2126 2048 2048 2048 2048 2048 s2000`
  - from stand (deg): R_hroll=-11.3 R_hpitch=-18.5 R_k=-40.7 R_a=-22.1 L_hroll=-11.3 L_hpitch=-18.5 L_k=-40.7 L_a=-22.1 R_ar=+11.3 L_ar=+11.3
  - from crouch (deg): R_hroll=-11.3 R_hpitch=+6.1 R_k=+8.5 R_a=+2.5 L_hroll=-11.3 L_hpitch=+6.1 L_k=+8.5 L_a=+2.5 R_ar=+11.3 L_ar=+11.3
- rung 4: x1.1 = 68.2 mm (19:05:33.740Z): `pose 2224 1799 2565 1803 2133 1860 2417 1849 2184 2153 2250 2139 2048 2048 2048 2048 2048 s2000`
  - from stand (deg): R_hroll=-12.5 R_hpitch=-17.4 R_k=-38.7 R_a=-21.3 L_hroll=-12.5 L_hpitch=-17.4 L_k=-38.7 L_a=-21.3 R_ar=+12.5 L_ar=+12.5
  - from crouch (deg): R_hroll=-12.5 R_hpitch=+7.2 R_k=+10.6 R_a=+3.3 L_hroll=-12.5 L_hpitch=+7.2 L_k=+10.6 L_a=+3.3 R_ar=+12.5 L_ar=+12.5
- rung 5: x1.2 = 74.4 mm (19:05:42.735Z): `pose 2236 1814 2539 1814 2145 1875 2391 1860 2184 2153 2262 2151 2048 2048 2048 2048 2048 s2000`
  - from stand (deg): R_hroll=-13.5 R_hpitch=-16.1 R_k=-36.4 R_a=-20.3 L_hroll=-13.5 L_hpitch=-16.1 L_k=-36.4 L_a=-20.3 R_ar=+13.5 L_ar=+13.5
  - from crouch (deg): R_hroll=-13.5 R_hpitch=+8.5 R_k=+12.8 R_a=+4.3 L_hroll=-13.5 L_hpitch=+8.5 L_k=+12.8 L_a=+4.3 R_ar=+13.5 L_ar=+13.5
- rung 6: x1.3 = 80.6 mm (19:05:51.804Z): `pose 2249 1831 2509 1827 2158 1892 2361 1873 2184 2153 2275 2164 2048 2048 2048 2048 2048 s2000`
  - from stand (deg): R_hroll=-14.7 R_hpitch=-14.6 R_k=-33.8 R_a=-19.2 L_hroll=-14.7 L_hpitch=-14.6 L_k=-33.8 L_a=-19.2 R_ar=+14.7 L_ar=+14.7
  - from crouch (deg): R_hroll=-14.7 R_hpitch=+10.0 R_k=+15.5 R_a=+5.5 L_hroll=-14.7 L_hpitch=+10.0 L_k=+15.5 L_a=+5.5 R_ar=+14.7 L_ar=+14.7
- rung 7: x1.4 = 86.8 mm (19:06:00.793Z): `pose 2261 1850 2474 1843 2170 1911 2326 1889 2184 2153 2287 2176 2048 2048 2048 2048 2048 s2000`
  - from stand (deg): R_hroll=-15.7 R_hpitch=-12.9 R_k=-30.7 R_a=-17.8 L_hroll=-15.7 L_hpitch=-12.9 L_k=-30.7 L_a=-17.8 R_ar=+15.7 L_ar=+15.7
  - from crouch (deg): R_hroll=-15.7 R_hpitch=+11.7 R_k=+18.5 R_a=+6.9 L_hroll=-15.7 L_hpitch=+11.7 L_k=+18.5 L_a=+6.9 R_ar=+15.7 L_ar=+15.7
- excluded per rung: lift 5 mm + lower (see json `excluded`); tail: 19:06:22.947Z `2082 1717 2685 1765 1991 1778 ...`; 19:06:26.548Z `2082 1997 2125 2045 1991 2058 ...`

## Flags / inconsistencies
- A shift pose is NOT roll-only: vs the crouch, the pitch chain also moves (both legs), e.g. R ladder x1.4: hip_pitch +11.7, knee +18.5, ankle +6.9 deg on BOTH legs (x0.5: ~+2 deg), the same on the L ladder by mirror. Hip roll and ankle roll are equal and opposite on both legs (hip +/-15.7, ankle -/+15.7 at x1.4; 0.18 deg/mm), so the foot stays flat. Yaw and unfitted slots unchanged. The extra pitch is the shift IK keeping pelvis height; expect it.
- Crouch itself: hip_pitch -24.6, knee -49.2, ankle -24.6 deg from stand on both legs; no roll change.
- Rungs are chained shift->shift (lower returns to the same shift pose, not to crouch); today's repeat from crouch each time is a different path.
- R ladder: 41 s gap between crouch and first shift (18:55:43 -> 18:56:31, probe/pos reads in between); L ladder 3.5 s. Log has `balance` commands only as status reads (state off); no B token (grep 0).
- 10-25 shift mm values are 62 x multiplier, not read from the log (tick deltas scale consistently: 65,98,129,142,154,167,179 R hip-roll ticks).
