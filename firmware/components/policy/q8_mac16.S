// q8RowsMac16 -- the int8 x int16 dot-product kernel on the ESP32's MAC16
// unit (policy/q8.h). Bit-identical to q8RowsC: policy::init() compares the
// two on the robot and forward() only runs this one if they agree.
//
//   void q8RowsMac16(const int16_t* x, const int8_t* w, int in_pad, int out,
//                    int32_t* acc);
//     acc[j] = sum_{i < in_pad} x[i] * w[j * in_pad + i]
//
// MAC16 multiplies 16-bit HALVES of registers into a 40-bit accumulator. The
// activations are int16 already: two per word, loaded straight into the MAC16
// data registers by the multiply instructions themselves (.ldinc). The int8
// weights are not 16-bit halves, so each word of four is split with one shift
// and two masks into two words whose halves hold w << 8 -- exact signed
// int16 values, 256 times the weight:
//
//   W           = [w3 w2 w1 w0]           (w0 in bits 0..7)
//   W & M       : lo = w1 << 8, hi = w3 << 8      M = 0xFF00FF00
//   (W << 8) & M: lo = w0 << 8, hi = w2 << 8
//
// so ACC = 256 * sum; |sum| < 512 * 32767 * 127 < 2^31 keeps 256 * sum inside
// the 40-bit accumulator, and the exact sum is (ACCHI:ACCLO) >> 8.
//
// Per four MACs: one weight load, three ALU ops, four MULA (two carrying the
// next activation words) and a pointer bump -- 9 instructions in a
// zero-overhead loop, ~2.3 cycles per MAC before memory stalls.
//
// Contract (q8.h): in_pad is a multiple of 4, >= 4; x and w are 4-byte
// aligned; both may be read one word past their end (x's scratch and the
// generated/copied weight arrays carry that slack). The weight-word prefetch
// and the activation pre-load are what read past the end.
//
// MAC16 state (ACCLO/ACCHI, M0..M3) is saved by the FreeRTOS port's
// interrupt/context frame (xchal_ncp_store), so preemption is safe.
#if defined(__XTENSA__)

    .section .iram1.q8_mac16, "ax"
    .align  4
    .global q8RowsMac16
    .type   q8RowsMac16, @function
q8RowsMac16:
    entry   a1, 32
    beqz    a5, .Ldone              // out == 0
    srli    a4, a4, 2               // words per row = in_pad / 4
    movi    a7, 0xFF                // a7 = 0xFF00FF00
    slli    a7, a7, 8
    slli    a8, a7, 16
    or      a7, a7, a8
    movi    a9, 0
    ssai    8                       // SAR = 8 for the final funnel shift

.Lrow:
    wsr     a9, ACCLO
    wsr     a9, ACCHI
    addi    a10, a2, -4             // .ldinc pre-increments
    ldinc   m0, a10                 // m0 = x[1]:x[0], a10 = x
    l32i    a11, a3, 0              // first weight word of the row
    loopnez a4, .Lrow_end
    slli    a12, a11, 8
    and     a13, a11, a7            // lo = w1<<8, hi = w3<<8
    l32i    a11, a3, 4              // next weight word (prefetch)
    and     a12, a12, a7            // lo = w0<<8, hi = w2<<8
    addi    a3, a3, 4
    mula.da.ll.ldinc m1, a10, m0, a12   // += x0*w0 ; m1 = x[3]:x[2]
    mula.da.hl       m0, a13            // += x1*w1
    mula.da.lh.ldinc m0, a10, m1, a12   // += x2*w2 ; m0 = next x[1]:x[0]
    mula.da.hh       m1, a13            // += x3*w3
.Lrow_end:
    rsr     a14, ACCLO
    rsr     a15, ACCHI
    src     a14, a15, a14           // (ACCHI:ACCLO) >> 8 = the exact sum
    s32i    a14, a6, 0
    addi    a6, a6, 4
    addi    a5, a5, -1
    bnez    a5, .Lrow

.Ldone:
    retw

    .size   q8RowsMac16, . - q8RowsMac16

#endif
