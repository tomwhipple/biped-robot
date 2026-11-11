import sys,subprocess
name,cam,t0,t1,n,crop,out,cols=sys.argv[1],sys.argv[2],float(sys.argv[3]),float(sys.argv[4]),int(sys.argv[5]),sys.argv[6],sys.argv[7],int(sys.argv[8])
T_CMD=1794356980.60   # host utc of the first vx=0.2 frame (arm log: script t=2.0 s)
pts=[float(x)/1000 for x in open(f'{name}_{cam}.pts').read().split('\n')[1:] if x.strip()]
want=[T_CMD+t0+i*(t1-t0)/(n-1) for i in range(n)]
idx=[min(range(len(pts)),key=lambda k:abs(pts[k]-w)) for w in want]
sel='+'.join(f'eq(n\\,{i})' for i in idx)
rows=(n+cols-1)//cols
lab=' '.join(f"{pts[i]-T_CMD:+.2f}" for i in idx)
vf=f"select='{sel}',crop={crop},scale=480:-1,tile={cols}x{rows}"
subprocess.run(['ffmpeg','-hide_banner','-loglevel','error','-i',f'{name}_{cam}.mp4','-vf',vf,'-vsync','0','-frames:v','1','-y',out],check=True)
print(out,'t rel cmd:',lab)
