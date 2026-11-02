# 2026-11-02 — a (256, 256) policy inside the 50 Hz tick: int8 on the ESP32

**Question.** The distillation study that morning put most of the student gap
on width: on walk14, students of (128, 128) / (256, 256) / (512, 256, 128)
score 37 / 62 / 69 of 160 on the python column, against 79 for the teacher.
The v6legs (128, 128) float net costs 7.8 ms of a 13.2 ms worst tick
(robot, 2026-11-01, `stat`). Can a (256, 256) net run on the ESP32 inside
the tick, and how?

**Answer.** Yes, as **int8 weights streamed from flash with a MAC16 kernel**:
an estimated ~5.4 ms per forward pass, *less* than the (128, 128) float net
costs today, for a worst tick of ~10.8 ms. It needs no RAM, which the image
does not have to spare. This is implemented as the policy profile `v6legs256`,
with a boot self-test and a `nettime` command that measures the estimates on
the robot. Numbers below are estimates from the generated code until
`nettime` runs. The design is
[firmware-design.md §6.1](../firmware-design.md); this page is the evidence.

## Where the time goes

- The net is `constexpr` in flash `.rodata`. The ESP32 reads it through a
  32 KB per-core flash cache (32-byte lines) over QIO SPI at 80 MHz, 40 MB/s
  raw: one 32-byte line takes ~84 SPI clocks, ~1.05 µs, plus the cache's
  own handling. The (128, 128) net's 162 KB of weights do not fit, so every
  tick streams them.
- Compute: the float loop compiles (GCC 14.2, `-O2`) to
  `lsi; lsi; addi; madd.s; ssi; addi` in a zero-overhead loop, 6
  instructions per MAC. The `madd.s → ssi` dependency adds stalls, so it is
  6–10 cycles per MAC, 1.0–1.7 ms for 40 576 MACs.
- Swish through newlib's `expf`: `expf` → `__ieee754_expf` (float-only,
  ~60 instructions, itself calling `__divsf3`) + `finitef`, then another
  `__divsf3` for the quotient. That is ~200 cycles a unit, 0.2 ms for 256.
- What remains of the measured 7.8 ms is the flash stream: ~6 ms for 162 KB,
  **~37 µs per KB (36–40)**, consistent with the SPI arithmetic above.
- Memory: static DRAM is 155 KB of the 181 KB segment (`.bss` 136 KB). The
  robot reports 105 KB of free heap with WiFi up
  (`hw_sessions/2026-11-01`, `rec status`). A float (256, 256) net is
  445 KB, and even its int8 form (109 KB) does not fit in RAM with a margin
  for WiFi.

## Options, estimated (ms per forward pass, 240 MHz)

| net | storage, kernel | bytes | flash | compute | **net** | tick |
|---|---|---|---|---|---|---|
| 128² | float, flash (deployed) | 162 KB | ~6.1 | 1.3–2.0 | **7.8 (measured)** | 13.2 (measured) |
| 128² | int8 MAC16, flash | 39 KB | 1.5 | 0.5 | ~2.0 | ~7.4 |
| 192² | float, flash | 286 KB | 10.6 | 2.6 | ~13 | ~18.5, no margin |
| 192² | int8 MAC16, flash | 71 KB | 2.6 | 0.9 | ~3.6 | ~9.0 |
| 256² | float, flash | 445 KB | 16.5 | 4.2 | ~21 | does not fit |
| 256² | float, both cores | 445 KB | 16.5 | 2.1 | ~19 | does not fit |
| 256² | int16 weights, MAC16 `mula.dd` | 218 KB | 8.1 | 0.7 | ~9 | ~14.5 |
| 256² | int8, C kernel, flash | 109 KB | 4.0 | 3.1 | ~7.1 | ~12.5 |
| 256² | **int8, MAC16, flash** | 109 KB | 4.0 | 1.4 | **~5.4** | **~10.8** |
| 256² | int8, MAC16, RAM | 109 KB | 0 | 1.4 | ~1.4 | ~6.8 (no RAM for it) |

The compute column comes from the instruction counts of the generated code:

- C int8 kernel: 22 instructions per 4 MACs (`l8ui, sext, l16ui, mul16s, add`),
  ~5.5–6 cycles per MAC.
- MAC16 kernel: 9 instructions per 4 MACs, ~2.3–2.5 cycles per MAC.
- `swishFast`: ~45 instructions plus one `__divsf3` (31 instructions),
  ~100 cycles a unit, against ~200 for `expf`.
- Activation quantize/dequantize: ~10 cycles an element.

Rejected:

- **Two cores.** Both cores' caches fill over the one SPI flash bus, so a
  flash-bound net does not get faster. Core 0 carries WiFi, whose interrupts
  and tasks would put their jitter into the control tick, and the split
  costs three cross-core barriers per tick.
- **ESP-NN.** Its fast kernels are for the S3's SIMD; on the classic ESP32
  it is plain C.
- **ESP-DSP `dotprod_s16`.** The same MAC16 unit, but it needs int16 weights:
  twice the bytes, so it is flash-bound at ~9 ms.
- **IRAM as weight storage.** 31 KB free; it allows only 32-bit loads,
  which the MAC16 kernel uses but the C fallback does not.

## What was built

- **Int8 forward pass** (`policy/q8.h`, `q8.cpp`):
  - weights: int8, per-output-channel scales, output-major rows padded to 4;
  - activations: dynamic per-tensor int16, with an exact int32 sum;
  - polynomial swish;
  - the head keeps only its 12 mean logits.
- **MAC16 kernel** (`q8_mac16.S`, IRAM). Each word of four int8 weights is
  split by one shift and two masks into int16 halves holding w << 8, and
  feeds four `mula.da` instructions, two of which also load the next
  activation words. The 40-bit accumulator then holds 256 · sum exactly.
  Its semantics are pinned on the host by an instruction-level C emulation
  (`test_policy`). On the robot `policy::init()` compares it with the C
  kernel to the bit at every boot, and `forward()` uses it only if they
  agree.
- **Bit parity.** Every float step is one IEEE operation and `q8.cpp` builds
  with `-ffp-contract=off`, so the robot, the host/SIL build and the numpy
  mirror (`tools/policy_q8.py`) agree to the bit up to the final `tanhf`.
  The host golden vectors hold at 2e-6.
- **GPTQ rounding at export** (`gen_policy_weights.py --quant int8`). Inputs
  are rounded in turn with error feedback weighted by the layer's input
  covariance on 4 800 CPU-plant rollout observations of the float net. Frozen
  channels, which never move in calibration, are rounded first and take
  nobody's error. No retraining and no GPU; it takes ~10 s.
- **Boot.** `policy::init()` places what fits into RAM (free heap minus a
  64 KB WiFi reserve, largest layer first), runs the generator's self-test
  case, and refuses `run` if it fails.
- **`nettime [n]`** times every variant the image can run, on core 1 with
  the loop benched.
- **Small exact win on the float nets.** `forwardNet` computes only the head's
  mean half; a host test checks it is bit-identical. Nothing else in the
  float path changes.
- **Profile `v6legs256`.** The walk14 (256, 256) student
  `robot_legs_walk14_s256r24` as an int8 test net. It is legacy: it is there
  for timing and parity, and is not a deploy candidate. The image is
  `0xfe510` B (67 % of the app partition free). Against the float v6legs
  image, `.bss` grows 1.4 KB (the int8 scratch) and IRAM 1.3 KB (the hot
  int8 code). `nettime` takes its ~6 KB from the heap only while it runs, so
  the float images' `.bss` is unchanged (+0.1 KB).

## Parity

**Action error, int8 vs float**, on the s256 student:

| | rms | p99 | max |
|---|---|---|---|
| round-to-nearest, own rollouts | 0.007 | 0.021 | 0.062 |
| GPTQ, held-out seeds | 0.0035 | 0.011 | 0.029 |
| GPTQ, the 4 800 calibration obs | 0.0039 | 0.014 | 0.067 |
| GPTQ vs brax, 16 SIL golden cases (normal z up to 9σ) | | | 0.066 |

Of the activation error, int16 accounts for < 2e-4; the rest is weights. The
same scheme with round-to-nearest gives 0.006 rms / 0.044 max on the
deploy candidate `walk16_act_s128r24`, and 0.006 / 0.061 on the
(512, 256, 128) student.

Gates:

| check | result |
|---|---|
| host `test_policy`, v6legs256 | firmware int8 vs `policy_q8` mirror ≤ 2e-6 |
| host `test_sil_golden` | int8 vs brax within 0.08 |
| `pytest sim/sil` profile parity (closed loop) | SIL library vs the int8 mirror on the library's own obs < 1e-5; vs the training obs stacking < 5e-2 |

**Closed loop, SIL column** (the firmware stack on the CPU plant, 20
scenarios × 8 seeds, hardware-claim conditions), s256 student:

| | seed-pass | falls | stand_10s drift | stand_off drift |
|---|---|---|---|---|
| float (v6legs build, the `.silw` run float) | **87/160** | 1/160 (reversal) | 1.1 cm | 2.2 cm |
| int8 (v6legs256 build) | **86/160** | 1/160 (reversal) | 1.0 cm | 2.2 cm |

Per scenario the columns are identical except `turn_180`, 4/8 → 3/8, one seed.

## To do on the robot (no flash was done here)

1. Flash `make firmware POLICY=v6legs256` (never armed: it is a test net).
   The boot log line `policy net: int8, ...` gives placement, kernel and
   self-test.
2. Run `nettime 50`. Read off as run, C vs MAC16 from flash, RAM, and per
   layer flash vs RAM, which is the measured µs per KB.
3. Optionally, on the v6legs image, `nettime` times the deploy net as float
   from RAM, and as int8 (round-to-nearest, timing only).

If the per-layer flash/RAM pairs come in far from 37 µs/KB, the table above
moves with them; the ranking does not.
