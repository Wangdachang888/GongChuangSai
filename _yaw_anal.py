import math
from pathlib import Path
PV=0.073632
rows=[]
for line in Path(r"E:/LOG1/r20261031_1727081.csv").read_text(encoding="utf-8",errors="replace").splitlines():
    if not line.strip() or line.startswith("I"): continue
    p=line.split(",")
    if len(p)<43: continue
    try: rows.append([float(x) for x in p[:43]])
    except: pass
ms0=rows[0][5]

def wrap(a):
    while a>180: a-=360
    while a<=-180: a+=360
    return a

print("=== STEP1 turn (should go to ~-90) ===")
seg=[r for r in rows if int(r[2])==1 and int(r[3])==1]
for i,r in enumerate(seg):
    if i%5==0 or i==len(seg)-1 or abs(r[26])<3:
        print(f"t={(r[5]-ms0)/1000:6.2f} yaw={r[16]:7.2f} imu={r[17]:7.2f} th_tgt={r[23]:7.2f} herr={r[26]:6.2f} w_tgt={r[14]:6.3f} w_act={r[15]:6.3f} reached={int(r[4])} xy=({r[32]:.3f},{r[33]:.3f})")

print("\n=== STEP2 translate: yaw vs path heading ===")
seg=[r for r in rows if int(r[2])==1 and int(r[3])==2]
# path heading from displacement
x0,y0=seg[0][32],seg[0][33]
last=None
for i,r in enumerate(seg):
    t=(r[5]-ms0)/1000
    if last is None or t-last>=0.4 or i==len(seg)-1:
        # local path direction over ~0.3s window
        j0=max(0,i-6); j1=min(len(seg)-1,i+6)
        dx=seg[j1][32]-seg[j0][32]; dy=seg[j1][33]-seg[j0][33]
        path_yaw=math.degrees(math.atan2(dy,dx)) if (abs(dx)+abs(dy))>1e-4 else float("nan")
        body=r[16]
        skew=wrap(body - path_yaw) if path_yaw==path_yaw else float("nan")
        # intended body for step2 after clockwise 90 from +X is -90, path should be -Y i.e. path_yaw=-90
        print(f"t={t:6.2f} xy=({r[32]:7.3f},{r[33]:7.3f}) body={body:7.2f} imu={r[17]:7.2f} th_tgt={r[23]:7.2f} herr={r[26]:6.2f} path={path_yaw:7.2f} skew(body-path)={skew:6.2f} vt={r[19]*PV:4.2f} va={r[20]*PV:4.2f}")
        last=t

print("\n=== Yaw error stats during STEP2 cruise (err_xy>0.3) ===")
cruise=[r for r in seg if math.hypot(r[24],r[25])>0.3]
if cruise:
    herrs=[r[26] for r in cruise]
    bodies=[r[16] for r in cruise]
    imus=[r[17] for r in cruise]
    print(f"n={len(cruise)} body mean={sum(bodies)/len(bodies):.2f}  imu mean={sum(imus)/len(imus):.2f}")
    print(f"herr mean={sum(herrs)/len(herrs):.2f} min={min(herrs):.2f} max={max(herrs):.2f}")
    print(f"th_tgt unique={[round(r[23],2) for r in cruise[::max(1,len(cruise)//5)]]}")

print("\n=== Compare all translate steps: mean body yaw vs expected ===")
expected={0:0.0, 2:-90.0, 4:180.0, 6:90.0}
for step,exp in expected.items():
    seg=[r for r in rows if int(r[2])==1 and int(r[3])==step and math.hypot(r[24],r[25])>0.4]
    if not seg: continue
    bm=sum(r[16] for r in seg)/len(seg)
    im=sum(r[17] for r in seg)/len(seg)
    hm=sum(r[26] for r in seg)/len(seg)
    # path direction
    dx=seg[-1][32]-seg[0][32]; dy=seg[-1][33]-seg[0][33]
    path=math.degrees(math.atan2(dy,dx))
    print(f"step{step}: expected_body={exp:7.1f} mean_body={bm:7.2f} mean_imu={im:7.2f} mean_herr={hm:6.2f} path_heading={path:7.2f} skew={wrap(bm-path):6.2f}")
