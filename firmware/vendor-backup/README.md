# Vendor firmware backup

`waveshare_stock_4MB.bin` is a full 4 MB dump of the Waveshare "Servo Driver
with ESP32" flash **exactly as it shipped**, taken 2026-07-26 before our
firmware was flashed for the first time.

    sha256  5ef7db3e1ea47654016b060016979e3e4b9758780dd5bf3262e1d1ac54f6db0f
    size    4194304 bytes
    chip    ESP32-D0WD-V3 rev 3.1, MAC 28:05:a5:c2:a6:20

**The .bin is deliberately not in git** (4 MB binary — see `../.gitignore`).
If it is missing and the board still runs vendor firmware, retake it:

```
. ~/esp/esp-idf/export.sh
esptool.py --port /dev/cu.usbserial-0001 -b 460800 \
    read_flash 0 0x400000 firmware/vendor-backup/waveshare_stock_4MB.bin
```

## Restoring it

```
. ~/esp/esp-idf/export.sh
esptool.py --port /dev/cu.usbserial-0001 -b 460800 \
    write_flash 0 firmware/vendor-backup/waveshare_stock_4MB.bin
```

That puts the stock web UI back at `192.168.4.1` (AP `ESP32_DEV` /
`12345678`), including its ID-setting page.

## Why you would, and mostly would not

Kept as insurance for the first hardware run of our firmware, not because the
vendor image is worth returning to. It is **actively wrong for these servos**:
the board ships the **SC-series** build, which is big-endian, while the ST3215
is ST-series little-endian — so every 16-bit value it exchanges is
byte-swapped. Measured on servo 9: it reported position 772 for an actual
1027, and its "Set Servo Mode" button wrote angle limits of 5120 and 60163
instead of 20 and 1003. Only 8-bit registers (ID, mode, torque) survive the
trip intact.

See [docs/bringup-day1.md](../../docs/bringup-day1.md) for the full measurement.
