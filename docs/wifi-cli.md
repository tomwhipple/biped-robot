# The Wi-Fi CLI

The robot's bench CLI -- `ping`, `pos`, `pose`, `balance`, `home`, `release`,
`torque`, `sread`, `imu`, `gains`, `rec`, ... -- over Wi-Fi, authenticated,
with the same text back that the USB tether prints. Wi-Fi is the primary
robot interface; the tether stays for flashing and for the writes that
persist (calibration, IDs, credentials, the key itself).

| piece | where |
|---|---|
| protocol, session logic, allowlist (pure C++, host-tested) | `firmware/components/wcli/` |
| the ESP32 glue: UDP 4213 socket, NVS key, output sink | `firmware/main/wifi_cli.cpp` |
| the wire format's reference, and the client session | `link/wcliproto.py` |
| one command / interactive / pipe | `tools/wcli.py` |
| the drop-in replacement for the tether FIFO bridge | `tools/wcli_bridge.py` |
| the robot half on a localhost socket (no robot needed) | `firmware/host/wcli_twin.cpp` |
| tests | `firmware/host/test_wcli.cpp`, `tests/test_wcli.py` |

## Provisioning (once, over the tether)

1. On mira, make the key (32 random bytes, a file only you can read):

   ```sh
   .venv/bin/python tools/wcli.py --keygen        # ~/.config/bimo/wcli.key, mode 600
   ```

   It refuses to overwrite an existing key, and prints the key's
   fingerprint and the provisioning line.

2. Put the key into the robot's NVS **over the USB tether**, benched (the
   CLI refuses `wcli key` over Wi-Fi). With the tether bridge running:

   ```sh
   echo "wcli key $(cat ~/.config/bimo/wcli.key)" > <board.fifo>
   echo "wcli" > <board.fifo>
   ```

   The robot answers `wcli: key stored in NVS, fingerprint xxxxxxxx`, and
   `wcli` shows the same fingerprint as `tools/wcli.py --fingerprint`. The
   robot never prints the key, and the flight recorder logs the line as
   `wcli key <redacted>` -- but the tether bridge's log does contain the hex
   (it logs what it sends). That log stays on mira; scrub the line if you
   like (`sed -i 's/wcli key [0-9a-fA-F]\{64\}/wcli key <redacted>/' <log>`).

3. To rotate: delete the key file, `--keygen` again, provision again (the new
   key drops any session). `wcli key clear` (tether) turns the Wi-Fi CLI off.

The robot needs Wi-Fi credentials already (`wifi <ssid> <psk>`, tether) and
listens on UDP 4213 at its DHCP address (192.168.2.90 on the bench network).

## Using it

```sh
.venv/bin/python tools/wcli.py ping                # one command, its reply, exit
.venv/bin/python tools/wcli.py 'pose ... s1500 BL' --linger 5
                                                   # ... and 5 s of async output
.venv/bin/python tools/wcli.py                     # interactive (or a pipe)
```

`--host` (default `$BIMO_HOST` or 192.168.2.90), `--key-file` (default
`$BIMO_WCLI_KEY` or `~/.config/bimo/wcli.key`). Exit status: 0 executed, 2
refused by the robot, 3 not executed (busy, or cancelled before it ran), 4
outcome unknown or timed out, 1 no session.

### In place of the tether FIFO bridge

Session scripts write lines into a FIFO and read a timestamped log. The Wi-Fi
bridge takes the same two arguments and writes the same log format:

```sh
.venv/bin/python tools/wcli_bridge.py /path/board.fifo /path/board.log &
echo "ping" > /path/board.fifo
```

```
17:44:11.250Z > ping                       a command, when it is sent
17:44:11.262Z   L_hip_yaw    id  1 ...     what the robot printed
17:44:11.300Z # ...                        the bridge's own notes
```

`__quit__` closes it, as on the tether. Point an existing script at the new
FIFO/log pair and it works unchanged -- with these differences, all of them
logged as `#` lines a script can stop on:

- A command that cannot be delivered now is **dropped, never queued**
  (`# WRITE FAILED (link down): <cmd>`), the tether bridge's rule.
- Commands run **one at a time, in FIFO order**: the bridge waits for each
  reply before sending the next line. A line that waited more than `--stale`
  seconds (default 10) behind a slow command is dropped
  (`# DROPPED (stale ...)`), not sent late.
- Delivery is confirmed, and every command runs **at most once**. When the
  outcome cannot be known (the session was lost after sending -- the robot
  rebooted), the bridge says `# OUTCOME UNKNOWN` and does **not** re-send.
- Output the robot could not re-send shows up as a reply line
  `[wcli: N chunk(s) of robot output lost]` -- a gap, never silence.
- The robot's async lines (balance ON/ABORT/landed, pose and home verdicts,
  link arm/home edges) arrive as reply lines whenever they happen, as on the
  tether.

A tether line's own reply goes to the tether only, and a Wi-Fi line's reply
to the Wi-Fi client only; the async lines go to both.

## What is refused over Wi-Fi

The Wi-Fi CLI is **bench-only**. While the control loop is armed (RUN, or
ctrl still holding the bus) only status reads are allowed -- `help`, `stat`,
`ntp`, and the no-argument forms of `wifi`, `batt`, `rec` (or `rec status`),
`cal` (or `cal show`), `balance`, `wcli`; everything else is refused with
"the control loop is armed and the Wi-Fi CLI is bench-only". Arming and
disarming stay with the commander (the ARM edge) and the tether.

Benched, the allowlist is **default-deny**: a command word not on it is
refused, so a CLI command added later is tether-only until someone adds it on
purpose (`firmware/components/wcli/policy.cpp`).

| allowed (benched) | refused over Wi-Fi (tether only), and why |
|---|---|
| `help` `scan` `ping` `pos` `move` `release` `torque` `pose` `balance` (incl. `set`, `log`, `off`) `reg` `sread` `trace` `home` `shape` `obsfreeze` `volt` `stat` `ntp` `bno` `bench` | `id` -- servo EEPROM |
| `rec ...` -- **RAM only**: over Wi-Fi a change is not saved to NVS | `middle` -- servo EEPROM |
| `gains`, `gains <id>` (reads) | `gains <id> <P> <D>` -- servo EEPROM |
| `cal`, `cal show` | every other `cal` form -- the calibration (live or NVS) |
| `imu`, `imu scan/raw/ring/avg/ae/reinit`, `imu gscale`, `imu mount` (reads) | `imu bias`, `imu gscale x y z`, `imu mount w x y z`, `imu forget` -- NVS |
| `wifi` (status) | `wifi <ssid> <psk>`, `wifi clear` -- credentials (NVS) |
| `wcli` (status) | `wcli key ...`, `wcli drop` -- the key itself |
| `batt` (status) | `batt reset` -- clears the pack-guard latch: swap the pack at the robot, reset on the tether (or reboot) |
| `stream status`, `stream dump` | `stream <id> ...` -- drives a servo with no torque gate and no envelope clamp |
| `obsdump`, `obsdump show/off` | `obsdump on/once` -- streams to the tether UART only |
| | `run` -- arming is the commander's ARM edge |
| | anything else -- not on the allowlist |

Two locks, not one: the allowlist refuses these lines before they reach the
CLI, and the CLI refuses them again itself (`cli.cpp tetherOnly()`) when the
line came over Wi-Fi -- a slip in one table does not reach NVS, servo EEPROM
or a safety latch.

## Safety

- **The same gates as the tether.** An allowed line runs through the very
  same `cli::execute()` on the housekeeping task: `claimBus` (bench mode, bus
  handed back), `torqueGate` (no blind goal writes), the mechanical-envelope
  clamp, `balLanding`. Tether and Wi-Fi lines share one queue and run one at
  a time, so a Wi-Fi line can never land inside another bus command, and the
  commander's ARM edge is still consumed only between lines. The Wi-Fi
  channel holds at most one slot of that 4-deep queue (one command in flight
  per session), so it cannot crowd out the tether. The bench-only
  check is made on the housekeeping task right before execution -- that task
  is the mode's writer, so nothing can arm the loop in between.
- **At most once.** A retransmitted command is answered, never re-run; a
  delayed copy of an older command is ignored; a command from an earlier
  session (or an earlier boot) is dead on arrival. A client that gives up on a
  command cancels it, which burns its sequence number: a copy still in flight
  can no longer run.
- **Wi-Fi drops mid pose stream: nothing new happens.** The stream runs on the
  robot (`poseTick`) and finishes as it would have, then holds; a balance hold
  (`pose ... B<L|R>`) keeps balancing, with its own tip/IMU abort, until the
  next command -- exactly as when the tether drops. No queued command runs
  late on reconnect: the client never re-sends into a new session.
- **No link-loss timeout ends a balance hold, by decision.** Ending the hold
  disengages the roll PI, which steps the stance leg's ankle/hip corrections
  to zero with the robot on one foot -- a motion caused by packet loss, worse
  than the hold it ends. Torque is never released by the link either (a
  released, loaded STS3215 back-drives). The hold is a bench state with a
  spotter; the robot's own abort (tip or IMU fault) stays the safety, and the
  tether has never had a link-loss timeout. If a timeout is wanted later, it
  should land the foot through the abort sequence (`abortTick`), not
  disengage.
- **A dead client cannot stall the robot.** Output waits for a live client's
  acknowledgements (up to 2 s per print -- the tether's UART blocks the same
  way when full); a client silent for 1 s is not waited for at all. Async
  output never waits: it runs inside the bench balance loop.

## Protocol

Every datagram, both directions, little-endian:

```
'B' 'W' | ver u8 = 1 | type u8 | sid u64 | seq u32 | aux u32 | flags u16 | len u16 | payload[len] | tag[16]
```

`tag` = HMAC-SHA256(direction key, everything before it), truncated to 16
bytes. From the 32-byte shared key K: client->robot messages use
HMAC(K, "bimo-wcli c2r v1"), robot->client messages HMAC(K, "bimo-wcli r2c
v1"), so a robot message reflected back at the robot fails its tag. The tag is
checked in constant time before any field is believed; a datagram that fails
is dropped and **never answered**. A CLI line is at most 192 bytes (the
tether's limit; a longer one is dropped as malformed), an output chunk at most
400 bytes.

| type | dir | meaning |
|---|---|---|
| 0x01 HELLO | c->r | sid = a fresh client nonce |
| 0x81 CHALLENGE | r->c | sid = a fresh random session id (hardware RNG), payload = the client nonce |
| 0x04 POLL | c->r | aux = the next output seq wanted (acknowledges all below); flag RESEND = re-send from aux. Also the keepalive |
| 0x02 CMD | c->r | seq = command seq, payload = the line |
| 0x03 CANCEL | c->r | seq = a command the client gave up on |
| 0x83 STATUS | r->c | seq = next output seq, aux = the highest command seq consumed, flags = its state (queued/running/done/cancelled) + BUSY (this CMD or pairing was not accepted) + LAGGING (output was overwritten), payload = oldest output seq still held |
| 0x82 OUT | r->c | seq = output seq, aux = the command it belongs to (0 = async), flags DONE / ASYNC / REFUSED, payload = CLI text |
| 0x84 STALE | r->c | sid = the session id the client used, which is not current: HELLO again |

**Pairing.** HELLO -> CHALLENGE -> POLL -> STATUS. A challenge is bound to the
address that asked and expires in 10 s. The first authenticated message
carrying it makes it *the* session (one paired client; the latest wins, and
the previous client is told STALE) -- but never while a command is queued or
running. A HELLO alone changes nothing, so a replayed HELLO only earns a
challenge the replayer cannot use. Within a session only the paired address
is accepted.

**Commands.** Seqs start at 1 per session and strictly increase. A new seq
while another command is in flight, or with the CLI queue full, gets STATUS
BUSY and is *not* consumed (the retransmit tries again). seq == last: STATUS,
no execution. seq < last: ignored. The client retransmits CMD every 250 ms
until STATUS acknowledges it; after 3 s without that it sends CANCEL until the
robot says whether the command was cancelled or had already started. A
CANCEL for a seq the robot has not seen burns it (nothing at or below it can
run later) without touching a command still in flight.

**Output.** The CLI's text, byte for byte, in chunks of up to 400 bytes with
one sequence for the whole session (command replies and async lines alike, in
the order the robot printed them). A command's last chunk carries DONE (and
REFUSED if it was refused); a command with no output still gets an empty DONE
chunk. The robot keeps the last 16 chunks; the client ACKs with POLL (every
100 ms while a command runs, every 2 s idle) and asks for gaps with
POLL+RESEND. Anything older than the oldest chunk held is reported lost.
Async output is kept only while a client has been heard from in the last 30 s.

## Threat model

**Assets.** The robot's motion (servo commands can break parts and hurt
people), its persistent configuration (calibration, servo EEPROM, gains, Wi-Fi
credentials, the key), and the integrity of what the bench log says happened.

**Attacker.** Anyone on the bench LAN or within radio range of the WPA2
network who can sniff, inject, replay, delay, reorder and spoof UDP datagrams
(any source address), and flood the robot. Not: someone with mira's shell,
the key file, or physical access to the robot (those own the robot anyway).

**Protected.**

- *Spoofed or forged commands*: every client->robot datagram must carry a
  valid HMAC-SHA256 tag under the shared key; nothing unauthenticated is
  executed or even answered (no oracle, no amplification).
- *Replay*: within a session, by the strictly increasing seq (a replayed CMD
  is at most answered, never re-run); across sessions and reboots, by the
  robot's fresh random session id -- a capture from an earlier session is
  answered STALE. A captured CMD replayed from another address is dropped.
- *Reflection*: direction-separated keys.
- *Spoofed replies*: robot->client datagrams are authenticated too, so a LAN
  attacker cannot make the client (or the bridge log) show text the robot did
  not print, nor fake a "done".
- *Persistent damage over the air*: the NVS/EEPROM writers and the pack-guard
  reset are refused over Wi-Fi twice over (allowlist + CLI); the key can only
  be set over the tether and is never sent over the air.
- *Motion while armed*: refused; the Wi-Fi CLI is bench-only.
- *Secrets in the flight recorder*: it ships CLI lines over UDP in clear, so
  `wcli key <hex>` and the PSK of `wifi <ssid> <psk>` are redacted before
  they are logged (this also covers tether lines).

**Not protected.**

- *Confidentiality.* Commands and replies are in clear (authenticated, not
  encrypted). Anyone on the LAN can read the bench session, as they can read
  the telemetry beacon and the flight recorder. The CLI prints no secrets
  (`wcli` shows only a 32-bit fingerprint of the key).
- *Denial of service.* A LAN attacker can jam Wi-Fi, deauthenticate the
  robot, flood port 4213 (each datagram costs one HMAC before it is dropped;
  the `wcli` task runs at the lowest priority so a flood starves only itself,
  but lwIP still has to receive the packets), or replay a captured HELLO to
  churn the two pending-challenge slots so a new pairing has to retry. Any
  key holder can take the session over (the latest pairing wins). Loss of the
  link never moves the robot (see Safety).
- *The commander link (UDP 4210)* is unauthenticated, as before: this channel
  does not change it. Arming and the ARM edge are its business, and its own
  watchdog is its safety (docs/control-channel.md).
- *A stolen key.* The key is in `~/.config/bimo/wcli.key` (0600) on mira, in
  the tether bridge's log after provisioning, and in the robot's NVS in clear
  (flash encryption is off; reading it needs physical access to the board).
  Whoever has it can do everything the Wi-Fi CLI allows -- which, by the
  table above, excludes persistent changes and anything while armed. Rotate
  by re-provisioning over the tether.
- *Truncated tags.* 128-bit tags: an online forgery needs ~2^127 tries.
- *Clocks.* Freshness does not use time at all (session ids and sequence
  numbers), so there is nothing to attack in SNTP here.

## Recorded

Every Wi-Fi line -- executed or refused -- is a flight-recorder `cmd` event
with `a = 1` (the channel; 0 is the tether) and `b` = its command seq, and
each line it printed an `out` event with `a = 1`; a new pairing is a `wcli`
event with the client's address (docs/flight-recorder.md).

## Tested, and not yet

Host-tested (`make host`, `pytest tests/test_wcli.py`): SHA-256 and HMAC
against the FIPS 180-2 and RFC 4231 vectors; the wire format byte for byte
against the Python reference; every single-bit flip rejected; no reply to
anything unauthenticated; reflection; replay within and across sessions and
from another address; at-most-once under 30 % packet loss in both directions;
cancel; busy; the output window, re-send and lost-chunk reporting; the
allowlist table; the tools' exit codes and the bridge's log format, against
the real session logic on a socket (`wcli_twin`).

Not testable without the robot: the ESP32 glue (socket, NVS key storage,
`esp_fill_random`), the housekeeping integration (the bench-only gate against
a real arm, the refusal of persistent writers by the CLI itself, async output
from `poseTick`), timing and throughput on the real radio, and the housekeeping
stack and heap margins (`stat` prints both; the `wcli` task's static cost is
about 12 KB of DRAM).
