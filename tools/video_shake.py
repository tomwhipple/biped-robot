"""Vibration meter from a 30 fps video: mean absolute frame difference inside a
region (default: centre band where the robot stands), per frame, then the
spectrum of that signal. Sees what a 10 Hz gyro burst cannot (5-15 Hz shake).
    .venv/bin/python tools/video_shake.py clip.mp4 [--roi x0,y0,x1,y1 fractions]"""
import sys, argparse, subprocess, numpy as np
ap=argparse.ArgumentParser(); ap.add_argument('video'); ap.add_argument('--roi', default='0.30,0.15,0.70,1.0'); ap.add_argument('--fps', type=float, default=30.0)
A=ap.parse_args()
# decode to grayscale raw via ffmpeg
probe=subprocess.run(['ffprobe','-v','error','-select_streams','v:0','-show_entries','stream=width,height','-of','csv=p=0',A.video],capture_output=True,text=True).stdout.strip().split(',')
W,H=int(probe[0]),int(probe[1])
raw=subprocess.run(['ffmpeg','-loglevel','error','-i',A.video,'-f','rawvideo','-pix_fmt','gray','-'],capture_output=True).stdout
n=len(raw)//(W*H); frames=np.frombuffer(raw[:n*W*H],dtype=np.uint8).reshape(n,H,W).astype(np.float32)
x0,y0,x1,y1=[float(v) for v in A.roi.split(',')]; roi=frames[:,int(y0*H):int(y1*H),int(x0*W):int(x1*W)]
d=np.abs(np.diff(roi,axis=0)).mean(axis=(1,2))            # per-frame motion energy
t=np.arange(len(d))/A.fps
# spectrum of the motion signal (detrended)
sig=d-d.mean(); spec=np.abs(np.fft.rfft(sig*np.hanning(len(sig)))); f=np.fft.rfftfreq(len(sig),1/A.fps)
band=lambda a,b: float(np.sqrt((spec[(f>=a)&(f<b)]**2).sum()))
print(f"{A.video.split('/')[-1]}: {n} frames, {n/A.fps:.1f} s; motion energy mean {d.mean():.3f}  p95 {np.percentile(d,95):.3f}  max {d.max():.3f}")
print(f"  spectral energy  0.5-2 Hz {band(0.5,2):.1f} | 2-5 Hz {band(2,5):.1f} | 5-10 Hz {band(5,10):.1f} | 10-15 Hz {band(10,15):.1f}")
peak=f[1:][np.argmax(spec[1:])]; print(f"  dominant {peak:.2f} Hz")
# burst detection: moves are the sustained high-energy stretches; report each burst's tail
base=float(np.median(d)); thr=max(3*base, base+0.15); on=d>thr
bursts=[]; i=0
while i<len(on):
    if on[i]:
        j=i
        while j<len(on) and (on[j] or (j+int(0.4*A.fps)<len(on) and on[j:j+int(0.4*A.fps)].any())): j+=1
        if (j-i)/A.fps>=0.8: bursts.append((i,j))
        i=j
    else: i+=1
print(f"  baseline {base:.3f}; bursts (>= 0.8 s above {thr:.2f}): {len(bursts)}")
for i,j in bursts:
    tail=d[j:j+int(1.5*A.fps)]; quiet=d[j+int(2.5*A.fps):j+int(4.0*A.fps)]
    print(f"    move {i/A.fps:6.1f}-{j/A.fps:6.1f} s  peak {d[i:j].max():.2f}  tail(0-1.5s) mean {tail.mean():.3f} max {tail.max():.3f}  quiet(2.5-4s) {quiet.mean() if len(quiet) else float('nan'):.3f}")
# 1-s timeline so the descent/hold/ascent phases are visible
print("  per-second motion energy: "+" ".join(f"{d[int(k*A.fps):int((k+1)*A.fps)].mean():.2f}" for k in range(int(n/A.fps))))
