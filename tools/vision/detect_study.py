#!/usr/bin/env python3
"""Synthetic detectability study behind the tag-size choice: renders sheet tags at a
given pixel size (frontal / tilted / blurred, 4 rotations), runs the detector.
Run from the repo root: .venv/bin/python tools/vision/detect_study.py"""
import sys; sys.path.insert(0, __import__('os').path.dirname(__import__('os').path.abspath(__file__)))
import cv2, numpy as np
from tagdefs import tag_image, detect, make_detector
det=make_detector()
rng=np.random.default_rng(0)
def trial(tid,s,tilt,blur,noise,rot,det):
    # tag 8 modules -> s px outer square; quiet zone 1 module white, on mid-gray
    mod=16; img=np.full((10*mod,10*mod),255,np.uint8)
    img[mod:9*mod,mod:9*mod]=tag_image(tid,8*mod)
    S=10*mod
    q=s/8.0
    full=s+2*q
    # target canvas
    C=int(full*2+40); c=C/2
    src=np.float32([[0,0],[S,0],[S,S],[0,S]])
    # destination: square of size full, rotated by rot, foreshortened in y by cos(tilt), perspective
    ang=np.deg2rad(rot); R=np.array([[np.cos(ang),-np.sin(ang)],[np.sin(ang),np.cos(ang)]])
    pts=np.array([[-1,-1],[1,-1],[1,1],[-1,1]])*full/2
    pts=pts@R.T
    pts[:,1]*=np.cos(np.deg2rad(tilt))
    # perspective: far side smaller
    k=1-0.15*np.sin(np.deg2rad(tilt))*np.sign(-pts[:,1]); pts[:,0]*=np.where(k>0,k,1)
    dst=(pts+c+rng.uniform(-.5,.5,2)).astype(np.float32)
    M=cv2.getPerspectiveTransform(src,dst)
    bg=np.full((C,C),120,np.uint8)
    w=cv2.warpPerspective(img,M,(C,C),flags=cv2.INTER_AREA,borderMode=cv2.BORDER_CONSTANT,borderValue=120)
    mask=cv2.warpPerspective(np.full_like(img,255),M,(C,C))>0
    bg[mask]=w[mask]
    # use area downsample for realism
    out=bg.astype(np.float32)
    if blur>0: out=cv2.GaussianBlur(out,(0,0),blur)
    out+=rng.normal(0,noise,out.shape)
    out=np.clip(out,0,255).astype(np.uint8)
    d=detect(out,det)
    return tid in d
conds={'frontal':(0,0.7,3),'tilt60':(60,0.7,3),'tilt70':(70,0.7,3),'blur1.5':(0,1.5,3),'tilt60+blur1.2':(60,1.2,4)}
tids=[10,11,12,13,14,0,1,2,3,4,5]
for name,(tilt,blur,noise) in conds.items():
    res=[]
    for s in range(8,41):
        ok=sum(trial(t,s,tilt,blur,noise,rot,det) for t in tids for rot in (0,17,45,80))
        res.append((s,ok/(len(tids)*4)))
    # smallest s with >=95% for all larger
    mn=None
    for s,r in res:
        if all(rr>=.95 for ss,rr in res if ss>=s): mn=s;break
    print(name,'min size(95% & all larger):',mn, ' rates:',' '.join(f'{s}:{r:.2f}' for s,r in res if s%2==0 and s<=30))
print('--- variants')
import tagdefs
def mk(up,**kw):
    p=cv2.aruco.DetectorParameters()
    p.cornerRefinementMethod=cv2.aruco.CORNER_REFINE_APRILTAG
    p.minMarkerPerimeterRate=0.02;p.adaptiveThreshWinSizeMax=63;p.adaptiveThreshWinSizeStep=6;p.errorCorrectionRate=.6
    for k,v in kw.items(): setattr(p,k,v)
    return cv2.aruco.ArucoDetector(tagdefs.DICT,p)
import types
for label,kw in {'base':{}, 'ppc2':{'perspectiveRemovePixelPerCell':2}, 'ppc2,margin.05':{'perspectiveRemovePixelPerCell':2,'perspectiveRemoveIgnoredMarginPerCell':0.05},'ppc8':{'perspectiveRemovePixelPerCell':8,'perspectiveRemoveIgnoredMarginPerCell':0.2}}.items():
    d=mk(1,**kw)
    for cname in ('frontal','tilt60'):
        tilt,blur,noise=conds[cname]
        r=[]
        for s in (16,20,24,28,32,40):
            r.append(sum(trial(t,s,tilt,blur,noise,rot,d) for t in tids for rot in (0,17,45,80))/44)
        print(label,cname,' '.join(f'{x:.2f}' for x in r))
print('--- upscale')
d=mk(1,perspectiveRemovePixelPerCell=8,perspectiveRemoveIgnoredMarginPerCell=0.2)
orig=tagdefs.detect
def det_up(g,detector=None,up=2):
    big=cv2.resize(g,None,fx=up,fy=up,interpolation=cv2.INTER_CUBIC)
    out=orig(big,detector)
    return {k:v/up for k,v in out.items()}
globals()['detect']=lambda g,dd: det_up(g,dd,2)
for cname in ('frontal','tilt60','tilt70','blur1.5'):
    tilt,blur,noise=conds[cname]
    r=[]
    for s in (14,16,20,24,28,32,40,50):
        r.append(sum(trial(t,s,tilt,blur,noise,rot,d) for t in tids for rot in (0,17,45,80))/44)
    print('up2',cname,' '.join(f'{x:.2f}' for x in r))
