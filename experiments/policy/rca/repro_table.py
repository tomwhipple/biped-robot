"""The SIM REPRO table of the RCA doc: the 2026-11-01 takeover (recorded start pose, firmware
arm path, 10 s at the stand command) on the training and fitted plants, one actuator/sensor
model per row, 8 seeds each, fresh env per episode."""
import os, sys, json
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import ingredients as I, simh
L = dict(lag_hz=12)
M = dict(typed=simh.MEAS_SERVO, model=simh.JF_PLANT)
ROWS = {
    "1 as trained: shaper 10 Hz + servo lag 12 Hz": dict(servo=L),
    "2 + servo dead time 60 ms (3 ticks), no other change": dict(servo=dict(lag_hz=12, delay=3)),
    "3 + servo lag 2 Hz, no other change": dict(servo=dict(lag_hz=2)),
    "4 + set-point hysteresis 2 deg": dict(servo=dict(lag_hz=12, db_deg=2.0)),
    "5 + hard floor (contact tc 2 ms)": dict(servo=L, model=dict(floor_tc=0.002)),
    "6 + floor friction 0.3": dict(servo=L, model=dict(friction=0.3)),
    "7 + IMU noise x3": dict(servo=L, imu_noise_x=3.0),
    "8 joint-fit actuator (fast bracket)": dict(typed=simh.JF_SERVO, model=simh.JF_PLANT),
    "9 MEASURED actuator (stepresp + sweep)": dict(M),
    "10 measured + reg-58 dq (19 ms late, 50 steps/s)": dict(M, obs=dict(dq_delay=1, dq_quant=1)),
    "11 measured, loop gain x1.25": dict(M, out_gain=1.25),
    "12 sweep-only fit (kp .47, kd .03, compliance past the encoder)": dict(servo=dict(), model=dict(simh.FITTED_ACTUATOR)),
    "13 measured + firmware fix: gyro obs x0.5, shaper 6 Hz": dict(M, obs=dict(gyro_gain=0.5), shaper_hz=6.0),
    "14 measured x1.25 + firmware fix": dict(M, out_gain=1.25, obs=dict(gyro_gain=0.5), shaper_hz=6.0),
}
if __name__ == "__main__":
    rows = I.run(ROWS, ["train", "fit"], 8, jobs=int(os.environ.get("JOBS", 14)), out=os.path.join(HERE, "repro_table.json"))
    real, _ = __import__("takeover").real_metrics()
    tab = I.table(rows, real)
    open(os.path.join(HERE, "repro_table.md"), "w").write(tab + "\n")
    print(tab)
