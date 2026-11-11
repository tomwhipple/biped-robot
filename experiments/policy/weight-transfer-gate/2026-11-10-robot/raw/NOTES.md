# 2026-11-10 (local) / 2026-11-11Z: measurements for #271 (sim gate #259)

Robot: v6 legs, battery in, no tether, Wi-Fi CLI; flashed policy robot_legs_prec16_f075_js20_s128r24 (`stat`).
Floor: the brown board on carpet, the same board as the 11-09 stride (Tom, this session).
Cameras: EMEET /dev/video0 at floor level on the robot's LEFT; Sony A7III on the robot's RIGHT, knees down.
Calibration: `cal show` at 23:59Z equals the zeros in experiments/one-leg-stand/one_leg.py (all 12 joints).

## Readings
- 00:1xZ whole robot on the kitchen scale, battery in: **1621 g** (Tom). (11-01 kitchen-scale total without battery: 1538 g.)
- Plant totals for comparison: grade 1372 g, train 1507 g (shift_ladder_sim.md); both carry no pack.

## Log
- 00:06Z both feet on the one scale for the total: joints 3-10 deg off zero (legs squeezed in), loads 0.
- 00:08:33Z `home` #1 (robot on the bare board). MY ERROR: wcli returned at "streaming" and I sent `release` ~3 s later,
  during the hip play take-up: rolls left 10-39 ticks off zero (R hip roll -39, R ankle roll -32, L hip roll +17, L ankle roll +10).
  Clips home1_*.
- 00:09:15Z `home` #2 with --linger 14: "home readback: all 12 joints at their zeros -- torque RELEASED (ok)". Clips home2_*.
- 00:10Z robot placed (torque released, joints at zero): LEFT foot on the scale, right on a paper stack (Tom first said right, corrected; cameras agree: the scale foot is nearest the EMEET, on the robot's left). Scale: **792 g** of 1621 (L share 0.489). IMU roll +0.21 deg (bare board after home: +0.20).
- 00:13:33.173Z `stand`: pose 2082 1997 2125 2045 1991 2058 1977 2091 2184 2153 2108 1997 2048 2048 2048 2048 2048 s3000 -> id 52: torque is OFF, and a goal write would auto-enable it and MOVE the joint. `torque 52` first if you mean it. | up -0.0027 -0.0033 1.0000 | roll +0.19 deg | enc pos/load id52=2040/0 id11=2076/0 id51=2013/0 id12=2007/0
- 00:13:56Z stand pose refused (torque off, firmware guard); sent `torque` (broadcast, holds present position). Roll encoders before: 52=2040 11=2076 51=2013 12=2007 (i.e. the rolls sit 10-42 ticks off zero with torque released under the robot's weight, the same offsets as after home #1; home #2's readback had them at zero before the release).
- 00:14:00.871Z `stand`: pose 2082 1997 2125 2045 1991 2058 1977 2091 2184 2153 2108 1997 2048 2048 2048 2048 2048 s3000 -> smooth pose done (3012 ms) | up -0.0095 -0.0045 0.9999 | roll +0.25 deg | enc pos/load id52=2078/-32 id11=2097/-96 id51=1994/24 id12=2007/88
- 00:15:41Z Tom re-seated the robot (lifted 1 cm, set down, torque on at stand): roll +0.06 deg | enc pos/load id52=2081/-8 id11=2105/-32 id51=1992/8 id12=2001/40. No scale reading taken at this stand (take it on the way back).
- 00:15:46.610Z `crouch`: pose 2082 1717 2685 1765 1991 1778 2537 1811 2184 2153 2108 1997 2048 2048 2048 2048 2048 s3000 -> smooth pose done (3004 ms) | up -0.0203 0.0152 0.9997 | roll -0.87 deg | enc pos/load id52=2081/-8 id11=2105/-32 id51=1992/8 id12=2002/48

## Scale ladder (LEFT foot on the scale; total 1621 g)
| pose | scale g | L share | IMU roll deg | note |
|---|---|---|---|---|
| stand, torque on, re-seated | 817 | 0.504 | +0.06 | Tom |
| crouch | 771 | 0.476 | -0.87 | |
- 00:16:19.523Z `L05`: pose 2147 1739 2662 1766 2056 1800 2514 1812 2184 2153 2173 2062 2048 2048 2048 2048 2048 s2000 -> smooth pose done (2001 ms) | up -0.0296 0.0153 0.9994 | roll -0.88 deg | enc pos/load id52=2146/-8 id11=2168/-48 id51=2057/8 id12=2062/0
| L 0.5x (31.0 mm toward the scale foot) | 921 | 0.568 | -0.88 | |
- 00:16:49.588Z `L075`: pose 2180 1759 2632 1776 2089 1820 2484 1822 2184 2153 2206 2095 2048 2048 2048 2048 2048 s2000 -> smooth pose done (2018 ms) | up -0.0098 0.0134 0.9999 | roll -0.76 deg | enc pos/load id52=2180/0 id11=2201/-48 id51=2091/16 id12=2097/24
| L 0.75x (46.5 mm) | 1024 | 0.632 | -0.76 | |
- 00:17:28.050Z `L10`: pose 2211 1786 2588 1793 2120 1847 2440 1839 2184 2153 2237 2126 2048 2048 2048 2048 2048 s2000 -> smooth pose done (2018 ms) | up -0.0084 0.0095 0.9999 | roll -0.54 deg | enc pos/load id52=2211/0 id11=2233/-40 id51=2123/24 id12=2128/24
| L 1.0x (62.0 mm) | 1127 | 0.695 | -0.54 | |
- 00:17:58.580Z `L11`: pose 2224 1799 2565 1803 2133 1860 2417 1849 2184 2153 2250 2139 2048 2048 2048 2048 2048 s2000 -> smooth pose done (2011 ms) | up -0.0072 0.0082 0.9999 | roll -0.47 deg | enc pos/load id52=2224/0 id11=2246/-40 id51=2136/24 id12=2142/32
| L 1.1x (68.2 mm) | 1157 | 0.714 | -0.47 | |
- 00:18:24.404Z `L12`: pose 2236 1814 2539 1814 2145 1875 2391 1860 2184 2153 2262 2151 2048 2048 2048 2048 2048 s2000 -> smooth pose done (2010 ms) | up -0.0103 0.0077 0.9999 | roll -0.44 deg | enc pos/load id52=2236/0 id11=2258/-40 id51=2148/24 id12=2156/48
| L 1.2x (74.4 mm) | 1194 | 0.737 | -0.44 | |
- 00:18:39.199Z `L13`: pose 2249 1831 2509 1827 2158 1892 2361 1873 2184 2153 2275 2164 2048 2048 2048 2048 2048 s2000 -> smooth pose done (2018 ms) | up -0.0118 0.0053 0.9999 | roll -0.30 deg | enc pos/load id52=2249/0 id11=2272/-32 id51=2161/24 id12=2166/24
| L 1.3x (80.6 mm) | 1249 | 0.771 | -0.30 | |
- 00:18:54.405Z `L14`: pose 2261 1850 2474 1843 2170 1911 2326 1889 2184 2153 2287 2176 2048 2048 2048 2048 2048 s2000 -> smooth pose done (2004 ms) | up -0.0120 0.0040 0.9999 | roll -0.24 deg | enc pos/load id52=2261/0 id11=2284/-32 id51=2173/24 id12=2180/40
| L 1.4x (86.8 mm) | 1285 | 0.793 | -0.24 | |
- 00:19:14.552Z `crouch2`: pose 2082 1717 2685 1765 1991 1778 2537 1811 2184 2153 2108 1997 2048 2048 2048 2048 2048 s3000 -> smooth pose done (3008 ms) | up -0.0222 0.0188 0.9996 | roll -1.08 deg | enc pos/load id52=2081/-8 id11=2109/0 id51=1992/8 id12=2009/104
| crouch (after the L ladder) | 795 | 0.490 | -1.08 | +24 g vs the first crouch; L ankle roll enc 2009 (zero 1997) load 104 |
- 00:20:07.801Z `R05`: pose 2017 1739 2662 1766 1926 1800 2514 1812 2184 2153 2043 1932 2048 2048 2048 2048 2048 s2000 -> smooth pose done (2009 ms) | up -0.0225 0.0133 0.9997 | roll -0.77 deg | enc pos/load id52=2015/-16 id11=2040/-32 id51=1926/0 id12=1940/72
| R 0.5x (31.0 mm away from the scale foot) | 607 | 0.374 | -0.77 | |
- 00:20:31.299Z `R075`: pose 1984 1759 2632 1776 1893 1820 2484 1822 2184 2153 2010 1899 2048 2048 2048 2048 2048 s2000 -> smooth pose done (2019 ms) | up -0.0039 0.0122 0.9999 | roll -0.70 deg | enc pos/load id52=1982/-16 id11=2008/-24 id51=1893/0 id12=1909/88
| R 0.75x (46.5 mm) | 528 | 0.326 | -0.70 | |
- 00:21:05.923Z `R10`: pose 1953 1786 2588 1793 1862 1847 2440 1839 2184 2153 1979 1868 2048 2048 2048 2048 2048 s2000 -> smooth pose done (2007 ms) | up 0.0019 0.0082 1.0000 | roll -0.48 deg | enc pos/load id52=1950/-24 id11=1976/-32 id51=1862/0 id12=1875/64
| R 1.0x (62.0 mm) | 447 | 0.276 | -0.48 | |
- 00:21:22.419Z `R11`: pose 1940 1799 2565 1803 1849 1860 2417 1849 2184 2153 1966 1855 2048 2048 2048 2048 2048 s2000 -> smooth pose done (2004 ms) | up 0.0015 0.0069 1.0000 | roll -0.39 deg | enc pos/load id52=1937/-24 id11=1964/-24 id51=1849/0 id12=1861/56
| R 1.1x (68.2 mm) | 417 | 0.257 | -0.39 | |
- 00:21:39.413Z `R12`: pose 1928 1814 2539 1814 1837 1875 2391 1860 2184 2153 1954 1843 2048 2048 2048 2048 2048 s2000 -> smooth pose done (2008 ms) | up 0.0065 0.0029 1.0000 | roll -0.18 deg | enc pos/load id52=1925/-24 id11=1951/-32 id51=1837/0 id12=1849/56
| R 1.2x (74.4 mm) | 379 | 0.234 | -0.18 | |
- 00:21:56.009Z `R13`: pose 1915 1831 2509 1827 1824 1892 2361 1873 2184 2153 1941 1830 2048 2048 2048 2048 2048 s2000 -> smooth pose done (2008 ms) | up 0.0054 0.0017 1.0000 | roll -0.11 deg | enc pos/load id52=1912/-24 id11=1939/-24 id51=1824/0 id12=1837/64
| R 1.3x (80.6 mm) | 346 | 0.213 | -0.11 | |
- 00:22:16.184Z `R14`: pose 1903 1850 2474 1843 1812 1911 2326 1889 2184 2153 1929 1818 2048 2048 2048 2048 2048 s2000 -> smooth pose done (2005 ms) | up 0.0079 0.0007 1.0000 | roll -0.03 deg | enc pos/load id52=1900/-24 id11=1927/-24 id51=1812/0 id12=1825/64
| R 1.4x (86.8 mm) | 313 | 0.193 | -0.03 | |
- 00:22:34.545Z `crouch3`: pose 2082 1717 2685 1765 1991 1778 2537 1811 2184 2153 2108 1997 2048 2048 2048 2048 2048 s3000 -> smooth pose done (3009 ms) | up -0.0211 0.0100 0.9997 | roll -0.57 deg | enc pos/load id52=2080/-16 id11=2100/-72 id51=1992/8 id12=1996/0
| crouch (after the R ladder) | 742 | 0.458 | -0.57 | R ankle roll enc 2100 (zero 2108) load -72 |
- 00:22:58.474Z `stand2`: pose 2082 1997 2125 2045 1991 2058 1977 2091 2184 2153 2108 1997 2048 2048 2048 2048 2048 s3000 -> smooth pose done (3019 ms) | up -0.0037 -0.0068 1.0000 | roll +0.39 deg | enc pos/load id52=2081/-8 id11=2099/-80 id51=1992/8 id12=1996/0
| stand (after both ladders) | 799 | 0.493 | +0.39 | first stand 817 |
- 00:23:40Z `release` from the stand pose (no `home`: its hip play take-up would drag the feet on the paper). Ladder done.

### Ladder against the sim (L-foot share; sim from shift_ladder_sim.md, plants carry no pack: grade 1372 g, train 1507 g, robot 1621 g)
| pose | robot | grade | train | grade + ser_hip=14.2 |
|---|---|---|---|---|
| stand | 0.504 | 0.40 | 0.51 | 0.35 |
| crouch | 0.476 (0.490, 0.458 on return) | 0.41 | 0.51 | 0.35 |
| L 0.5 / 0.75 / 1.0 / 1.1 / 1.2 / 1.3 / 1.4 | 0.568 / 0.632 / 0.695 / 0.714 / 0.737 / 0.771 / 0.793 | 0.53 / 0.60 / 0.67 / 0.70 / 0.72 / 0.75 / 0.78 | 0.66 / 0.66 / 0.67 / 0.68 / 0.69 / 0.69 / 0.71 | 0.46 / 0.53 / 0.60 / 0.62 / 0.65 / 0.68 / 0.71 |
| R 0.5 / 0.75 / 1.0 / 1.1 / 1.2 / 1.3 / 1.4 | 0.374 / 0.326 / 0.276 / 0.257 / 0.234 / 0.213 / 0.193 | 0.31 / 0.27 / 0.24 / 0.22 / 0.21 / 0.20 / 0.19 | 0.35 / 0.34 / 0.34 / 0.32 / 0.31 / 0.31 / 0.30 | 0.28 / 0.26 / 0.24 / 0.23 / 0.23 / 0.22 / 0.21 |

Observed (not interpreted):
- The robot is laterally even at the stand (0.504 / 0.493); the grade plant puts 0.60 on the R foot there.
- Linear fits over the 7 rungs: L ladder share = 0.445 + 4.00e-3/mm, R ladder share = 0.477 - 3.26e-3/mm of commanded shift.
  Stance-foot share at 1.0x (62 mm): 0.695 (L), 0.724 (R); at 1.4x (86.8 mm): 0.793, 0.807. The other foot still carries 313-336 g at 1.4x.
- Crouch readings 771 / 795 / 742 g by approach direction: hysteresis about +-27 g (1.7 % of body weight).
- IMU roll stays within -1.1..+0.4 deg over the whole ladder.
Limits: both feet flat and position-held is statically indeterminate (the ankle-roll torques move load between the feet), so
this curve is a sim-validation target, not a direct CoM or mass-split measurement. Scale read by Tom, 1 g display. One pass per side.

## Stride repeat (item 1): prec16, stride1_02s, as on 11-09
- Firmware today = 11-09's (flight hello fw v1-hip60-backup-1212-g9d2a994-d in both records); policy robot_legs_prec16_f075_js20_s128r24.
- 11-09 command, read off prec16_run1_arm.log: arm_script.py --phase script --script stride1_02s --settle 3 --duration 5 --tail 0.5
  (deploy/prec16-f075 worktree ~/code/robot-deploy7, 5ac136b). 11-09 outcome: rate trip 3.02 at +0.64 s, WalkRecovery to the stand, no fall.
- Per-boot settings: at power-up today the robot read standclock run / shape 10 Hz (defaults). The 11-09 flight ticks show the
  stand clock held at 1.57, i.e. the prep had been run that day. 00:27Z: ran hw_sessions/2026-11-06/wifi_prep.sh 8
  (standclock coast 1.5708, shape 8). That 11-09 also had shape 8 is inferred from the prep script setting both, not read from the record.
- 00:29:13Z `home` #3 on the bare board before the stride: home readback: all 12 joints at their zeros -- torque RELEASED (ok); arm to walk | up -0.0053 -0.0106 0.9999 | bus 11.3 V (mean of 12 servos, 0.1 V resolution)
- 00:29:35Z **stride_r1**: arm_script exit 0, states bench -> live -> bench; `flail`: 1 walk recovery since boot (the rate guard tripped
  in the walk, as on 11-09). **Tom: "it almost fell and I caught it."** The beacon's up_z = 1.00 at the disarm is with Tom holding it;
  my first reading of the run as "standing" was wrong. After the run, before the home: up 0.0463 -0.0885 0.9950 (5.7 deg off vertical).
  Clips stride_r1_{emeet,sony}.mp4 (+ .pts), log stride_r1_arm.log, flight record hw_sessions/2026-11-09/flight_20261110T235251Z_prec16.frec.
- 00:30:32Z `home` #4 (Tom: "Home it"): home readback: all 12 joints at their zeros -- torque RELEASED (ok); arm to walk | up 0.0019 -0.0028 1.0000

### stride_r1 against 11-09 run1 (flight ticks, t from the first vx = 0.2 tick; flight_snapshot_0035Z_*.csv)
| | today | 11-09 |
|---|---|---|
| guard | rate 3.32 rad/s at +0.66 s -> WalkRecovery | rate 3.02 at +0.64 s |
| gyro_z peak | -7.26 rad/s at +0.62 s | -7.20 at +0.62 s |
| yaw (integrated gyro_z) at +0.5 / +0.7 / +1.0 / +2.0 s | +11 / -43 / -22 / -43 deg | +9 / -43 / -22 / -45 |
| max tilt (acos up_z) | 14.4 deg at +0.60 s (up_x +0.22, up_y +0.11) | 14.2 deg at +0.60 s |
| tilt at +0.3 / +0.5 s | 5.6 / 11.3 deg | 5.8 / 11.8 |
| R / L knee range | 48.2 / 59.5 deg | 47.6 / 57.1 |
| R / L hip roll range | 8.4 / 4.5 deg | 7.9 / 4.2 |
The run repeats 11-09 to within 1-2 deg and 0.1 rad/s.

### The feet (video; EMEET = floor level on the robot's LEFT, so its far foot is the RIGHT one; Sony on the right)
Sheets: stride_r1_emeet_C.jpg, stride_r1_emeet_Rfoot.jpg (zoom on the far foot), stride_r1_sony_C.jpg.
- +0.25..+0.38 s: both feet flat.
- from about +0.42 s the RIGHT foot's plate tilts: the end visible to the EMEET rises off the board (gap under it at +0.45); at +0.52..+0.59 it is
  raised and tilted roughly 10-15 deg, and by +0.59 it appears wholly off the board. Its other end is hidden behind the left foot, so "wholly off" is
  not established before +0.59. Sony, same times: the right foot tilted with one edge up, rotating.
- +0.59..+0.76 s: the LEFT foot is in the air (blurred, shadow under it) while the body yaws; the right foot is planted again, rotated about 45 deg.
- No height in mm: cam_record.sh defaults to 640x480 and I did not set SIZE, and there was no scale in frame. Next time SIZE=1920x1080.
So: on this repeat the right foot does leave the floor, tilted, for roughly 0.15 s, while the torso tips 11-14 deg (mostly about the body x axis of
`up`); the tilt comes before the yaw spike. The 11-09 note "the right foot never left the floor" is not what the floor camera shows on an identical run.

### The near-fall was after the disarm, not in the stride (stride_r1_sony_end.jpg)
The robot stands, yawed, from about +2 s to +4.4 s. The script's disarm is at +3.5 s; at +4.76 s the robot is toppling and by +5.0 s Tom has it.
arm_script --phase script ends with a bare ARM=0 disarm (torque off where it stands), not the reset/home edge. I did not check the script's ending
against the project rule that every armed run ends with a reset; that is my miss. 11-09 used the same ending and did not topple.

## Clamp test (item 3), 10-24 protocol with today's roll zeros
Rig: robot on the desk, RIGHT foot clamped to the desk edge (bar clamp on the toe end of the plate), left foot resting on the mat; Sony in front,
full-res stills (clamp/*.jpg), pelvis out of frame (torso roll from the IMU). Battery in. Pitch ticks of the lifts copied from
hw_sessions/2026-10-24/stand/left_serial.log (L 15 mm: 1816 2461 1849; L 30 mm: 1714 2666 1747).
- 00:40:49.449Z `c1_straight`: pose 2082 1997 2125 2045 1991 2058 1977 2091 2184 2153 2108 1997 2048 2048 2048 2048 2048 s3000 -> smooth pose done (3017 ms) | up 0.0094 0.0086 0.9999 | roll -0.50 (after still -0.51) | enc pos/load id52=2081/-8 id11=2095/-112 id51=1992/8 id12=2006/80
- 00:42:33.413Z `c2_straight`: pose 2082 1997 2125 2045 1991 2058 1977 2091 2184 2153 2108 1997 2048 2048 2048 2048 2048 s3000 -> smooth pose done (3018 ms) | up 0.0096 0.0089 0.9999 | roll -0.52 (after still -0.45) | enc pos/load id52=2082/0 id11=2104/-40 id51=1991/0 id12=2003/56
- 00:42:51.520Z `c3_liftL15`: pose 2082 1997 2125 2045 1991 1816 2461 1849 2184 2153 2108 1997 2048 2048 2048 2048 2048 s3000 -> smooth pose done (3020 ms) | up 0.0066 0.1319 0.9912 | roll -7.58 (after still -7.57) | enc pos/load id52=2078/-32 id11=2115/64 id51=1990/-8 id12=2002/48
- c1 straight (before re-clamp): stance (R) ankle roll 2095 (zero 2108) load -112 -> Tom loosened and re-tightened the clamp, and removed the rubber mat.
- c2 straight (baseline): roll -0.52 / -0.45 deg; enc 52=2082/0 11=2104/-40 51=1991/0 12=2003/56.
- c3 L foot commanded 15 mm up: roll **-7.58 deg** (7.1 deg toward the lifted side; 10-24: 4.1 at 15 mm, 5.0 at 30 mm). Stance enc: 11 2104 -> 2115
  (+0.97 deg, load 64), 52 2082 -> 2078 (-0.35 deg, load -32). asin(15/121) = 7.1 deg.
  **Tom: the left (lifted) foot is touching the desk on its outer edge; the clamped right foot is flat on the desk.** So the tilt is the
  geometric limit of the lift, not an equilibrium: the stance chain does not hold the torso inside 7.1 deg.
  Stills (angles.py, Hough edges, +-1 deg): stance shank 3.06 -> 6.80 (d 3.7), stance thigh 2.04 -> 7.61 (d 5.6), torso (IMU) d 7.1.
- 00:47:17.479Z `c4_liftL30`: pose 2082 1997 2125 2045 1991 1714 2666 1747 2184 2153 2108 1997 2048 2048 2048 2048 2048 s2000 -> smooth pose done (2002 ms) | up 0.0069 0.1772 0.9842 | roll -10.22 (after still -10.25) | enc pos/load id52=2074/-64 id11=2121/112 id51=1990/-8 id12=1997/0
- 00:48:01.953Z `c5_straight`: pose 2082 1997 2125 2045 1991 2058 1977 2091 2184 2153 2108 1997 2048 2048 2048 2048 2048 s2000 -> smooth pose done (2006 ms) | up 0.0086 0.0101 0.9999 | roll -0.57 (after still -0.60) | enc pos/load id52=2082/0 id11=2107/0 id51=1992/8 id12=2003/56
- c4 L foot commanded 30 mm up: roll **-10.22 deg**, steady (-10.25 after the still): 9.7 deg toward the lifted side (10-24, same rig side: 5.0 deg).
  Stance enc: 11 2104 -> 2121 (+17 ticks, 1.5 deg, load 112), 52 2082 -> 2074 (-8 ticks, 0.7 deg, load -64). 10-24: 11 gave 17 ticks at load 144,
  52 gave 10 ticks at load -80. So the encoders give what they gave on 10-24 and the tilt is 4.7 deg larger.
  Still c4: the lifted foot is off the desk (shadow under the whole plate). angles.py: stance shank 3.06 -> 8.69 (d 5.6), stance thigh 2.04 -> 9.68
  (d 7.6), torso d 9.7. Rough split (+-1 deg): foot->shank 5.6 (encoder 1.5, unseen ~4.1), shank->thigh ~2.0, thigh->torso ~2.1 (encoder 0.7, unseen ~1.4).
- 00:51Z Tom: "screws tightened a bit" (which screws: to be asked) -> repeat of the right-clamped test as d1..d4.
- 00:51:55.817Z `d1_straight`: pose 2082 1997 2125 2045 1991 2058 1977 2091 2184 2153 2108 1997 2048 2048 2048 2048 2048 s3000 -> smooth pose done (3009 ms) | up 0.0083 0.0101 0.9999 | roll -0.58 (after still -0.58) | enc pos/load id52=2082/0 id11=2103/-48 id51=1992/8 id12=2004/64
- 00:52:12.776Z `d2_liftL15`: pose 2082 1997 2125 2045 1991 1816 2461 1849 2184 2153 2108 1997 2048 2048 2048 2048 2048 s3000 -> smooth pose done (3011 ms) | up 0.0007 0.1274 0.9919 | roll -7.33 (after still -7.36) | enc pos/load id52=2078/-32 id11=2114/56 id51=1991/0 id12=2002/48
- 00:52:30.863Z `d3_liftL30`: pose 2082 1997 2125 2045 1991 1714 2666 1747 2184 2153 2108 1997 2048 2048 2048 2048 2048 s2000 -> smooth pose done (2011 ms) | up 0.0038 0.1726 0.9850 | roll -9.93 (after still -9.98) | enc pos/load id52=2074/-64 id11=2121/112 id51=1990/-8 id12=1998/0
- 00:52:48.040Z `d4_straight`: pose 2082 1997 2125 2045 1991 2058 1977 2091 2184 2153 2108 1997 2048 2048 2048 2048 2048 s2000 -> smooth pose done (2018 ms) | up 0.0066 0.0099 0.9999 | roll -0.57 (after still -0.57) | enc pos/load id52=2082/0 id11=2107/0 id51=1992/8 id12=2004/64
- Repeat after Tom tightened screws (right foot still clamped): d1 straight -0.58; d2 L 15 mm -7.33 (d 6.75); d3 L 30 mm -9.93 (d 9.35), steady;
  d4 straight -0.57. Stance enc at 30 mm: 11 2103 -> 2121, 52 2082 -> 2074, loads 112 / -64, as before. angles.py at 30 mm: stance shank d 6.5,
  thigh d 8.0 (the image method repeats to about +-1 deg: the two baselines differ by 0.9). Tightening changed the tilt by 0.35 deg: not the cause.

### Clamp test summary, right foot clamped, left lifted (today vs 10-24 `left_*`)
| lift | torso tilt today (run 1 / after tightening) | 10-24 | stance ankle roll enc give | stance hip roll enc give |
|---|---|---|---|---|
| 15 mm | 7.1 / 6.75 deg (lifted foot back on the desk, outer edge: Tom) | 4.1 | 11 ticks | 4 ticks |
| 30 mm | 9.7 / 9.35 deg, steady, foot in the air | 5.0 | 17 ticks (10-24: 17) | 8 ticks (10-24: 10) |
Differences from 10-24 that are known: battery in; the four roll zeros moved 2-3 deg after the 10-24 clamp runs (18:14Z fix: 52 2059 -> 2082,
11 2085 -> 2108, 51 2025 -> 1991, 12 2031 -> 1997), so "straight" is a different roll pose; the hip yoke broke and was repaired (11-02); a leg link
was replaced (11-06); desk without the mat, not perfectly level (Tom). Which of these accounts for the extra 4.4-4.7 deg is not established.
- Tom: tightened ALL the ankle screws (on the ankle servos and the servo wheels) before the d-series repeat.
- 01:04Z robot REBOOTED (battery alarm, Tom) and is now on BENCH POWER. LEFT foot clamped for the mirror test (e-series).
- 01:04:23.731Z `e1_straight`: pose 2082 1997 2125 2045 1991 2058 1977 2091 2184 2153 2108 1997 2048 2048 2048 2048 2048 s3000 -> smooth pose done (3013 ms) | up 0.0236 0.0115 0.9997 | roll -0.66 (after still -0.71) | enc pos/load id52=2082/0 id11=2107/0 id51=1992/8 id12=2008/96
- 01:06:09.059Z `e2_straight`: pose 2082 1997 2125 2045 1991 2058 1977 2091 2184 2153 2108 1997 2048 2048 2048 2048 2048 s3000 -> smooth pose done (3011 ms) | up 0.0194 0.0114 0.9997 | roll -0.65 (after still -0.65) | enc pos/load id52=2082/0 id11=2107/0 id51=1992/8 id12=2008/96
- 01:06:29.246Z `e3_liftR15`: pose 2082 1755 2609 1803 1991 2058 1977 2091 2184 2153 2108 1997 2048 2048 2048 2048 2048 s3000 -> smooth pose done (3015 ms) | up 0.0371 -0.0792 0.9962 | roll +4.54 (after still +4.55) | enc pos/load id52=2082/0 id11=2107/0 id51=1994/24 id12=1997/0
- 01:06:51.518Z `e4_liftR30`: pose 2082 1653 2814 1701 1991 2058 1977 2091 2184 2153 2108 1997 2048 2048 2048 2048 2048 s2000 -> smooth pose done (2017 ms) | up 0.0892 -0.1510 0.9845 | roll +8.71 (after still +8.73) | enc pos/load id52=2083/8 id11=2110/24 id51=1997/48 id12=1990/-64
- 01:07:13.496Z `e5_straight`: pose 2082 1997 2125 2045 1991 2058 1977 2091 2184 2153 2108 1997 2048 2048 2048 2048 2048 s2000 -> smooth pose done (2004 ms) | up 0.0171 0.0209 0.9996 | roll -1.21 (after still -1.19) | enc pos/load id52=2081/-8 id11=2105/-32 id51=1991/0 id12=1999/24
- Mirror (LEFT foot clamped, right lifted; bench power 12.0 V after the reboot). e1/e2 straight: roll -0.66 / -0.65; stance (L) ankle roll 12 at 2008
  (zero 1997) load 96 before AND after re-clamping (Tom: "nothing changed"); it read 2003-2007 with load 56-88 at the stand on the floor earlier.
- e3 R 15 mm: roll +4.54 (d 5.2 deg toward the lifted side; 10-24 `nz`: 4.4). Stance enc 12 2008 -> 1997 (load 0), 51 1992 -> 1994.
  Whether the lifted foot touched the desk was not checked.
- e4 R 30 mm: roll **+8.71** (d 9.36 deg), steady (+8.73); 10-24: 5.2. Stance enc 12 2008 -> 1990 (-18 ticks, 1.6 deg, load -64), 51 1992 -> 1997
  (+5 ticks, 0.44 deg, load 48). up_x went 0.019 -> 0.089 (about 4 deg of pitch as well; the other side showed none).
- e5 straight: roll -1.21 (0.56 deg left of the e2 baseline). angles.py gave too few lines on this side (fixed ROIs; the legs lean out of them): no split.

### Clamp test, both sides (torso tilt toward the lifted side, IMU)
| | 15 mm | 30 mm | 10-24 at 15 / 30 mm |
|---|---|---|---|
| right clamped, left lifted | 7.1 (6.75 after tightening) | 9.7 (9.35) | 4.1 / 5.0 |
| left clamped, right lifted | 5.2 | 9.36 | 4.4 / 5.2 |
Both sides give about 9.4 deg at 30 mm: the softness is symmetric, so it is not one damaged part.
- 01:08:50Z `release` from the straight stand on the clamp, Tom's hand on the robot (his go). Clamp test done.
