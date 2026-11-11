import cv2, numpy as np, sys, json
ROIS={'stance_shank':(2150,1450,3050,2550),'stance_thigh':(2150,0,3050,1150),'free_shank':(3000,1450,4000,2600),'free_thigh':(3000,0,4000,1150)}
def meas(path, rois=ROIS, dbg=None):
    im=cv2.imread(path); g=cv2.cvtColor(im,cv2.COLOR_BGR2GRAY); out={}
    vis=im.copy() if dbg else None
    for name,(x0,y0,x1,y1) in rois.items():
        r=cv2.GaussianBlur(g[y0:y1,x0:x1],(0,0),3)
        e=cv2.Canny(r,12,36)
        L=cv2.HoughLinesP(e,1,np.pi/1800,threshold=60,minLineLength=170,maxLineGap=30)
        ang=[];w=[]
        if L is not None:
            for xa,ya,xb,yb in np.asarray(L).reshape(-1,4).astype(int):
                if yb<ya: xa,ya,xb,yb=xb,yb,xa,ya
                a=np.degrees(np.arctan2(xb-xa,yb-ya))   # + = top of the edge is to the LEFT of its bottom?? (x grows down the edge)
                if abs(a)<25:
                    ang.append(-a); w.append(np.hypot(xb-xa,yb-ya))  # report + = edge leans to image-RIGHT going up
                    if dbg: cv2.line(vis,(x0+xa,y0+ya),(x0+xb,y0+yb),(0,255,0),4)
        if ang:
            ang=np.array(ang);w=np.array(w);o=np.argsort(ang);cw=np.cumsum(w[o]);med=ang[o][np.searchsorted(cw,cw[-1]/2)]
            out[name]=(round(float(med),2),len(ang),round(float(np.percentile(ang,25)),2),round(float(np.percentile(ang,75)),2))
        else: out[name]=None
        if dbg: cv2.rectangle(vis,(x0,y0),(x1,y1),(0,0,255),4)
    if dbg: cv2.imwrite(dbg,cv2.resize(vis,(1500,1000)))
    return out
if __name__=='__main__':
    for p in sys.argv[1:]:
        print(p, json.dumps(meas(p,dbg=p.replace('_sony.jpg','_lines.jpg'))))
