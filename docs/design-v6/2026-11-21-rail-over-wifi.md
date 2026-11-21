# The servo rail over Wi-Fi (decision, 2026-11-21)

**Decision (the owner, 2026-11-21, verbatim):** "change the rules. turning it on via wifi
should be allowed."

**Why.** With the power-lite board the servos have power only when the ESP32 closes K1. After
every power-up the Wi-Fi CLI refused `rail` (default deny, tether only), so someone had to
plug in USB only to give the servos power ([power-lite README](../../pcb/power-lite/README.md),
"The firmware").

**What changed** (`firmware/components/wcli/policy.cpp`, `firmware/main/cli.cpp`, the host
test `firmware/host/test_wcli.cpp`, the docs that list the Wi-Fi commands):

| over Wi-Fi | |
|---|---|
| `rail`, `rail on`, `rail off` | allowed |
| `rail clear` | tether only: acknowledging a STUCK RAIL fault is a hands-on step, and `rail on` never needs it (a stuck rail is refused until the fault is cleared, which no normal power-up reaches) |
| `rail panic` | tether only: a deliberate crash, the fail-safe's bench check |
| `bench` | allowed already (it clears the E-stop latch after a power cycle) |

Only the transport changed. `rail on` goes through `railEnsureUp` and `railguard` exactly as on
the tether: the E-stop latch, the pack guard's SAFE latch, NO RAIL / STUCK RAIL, the bus
handover, and refusal while armed. Armed, the Wi-Fi CLI is still status-only.

**Found on the same day: `home` over Wi-Fi with the rail off.** A typed `home` powered the rail
itself (`homePowered`), so K1 closed with every servo behind it at once, and the ESP32
rebooted (pack-sense read counter 2113 before, 203 a minute later; the owner heard K1 click).
The firmware had no record of why it reset. `home` keeps powering the rail itself (the
power-lite design: one command homes the servos); the reset repeated at the moment K1 closed
with the servos connected (pack-sense counter 2971 -> 135, INA219 12.11 V before and after;
cause not established). `rail` and `stat` now print the reset reason (`esp_reset_reason`:
power-on, brown-out, panic, task watchdog ...).
