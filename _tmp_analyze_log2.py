from pathlib import Path
p=Path(r"E:\LOG1\log1.csv")
rows=[]
for line in p.read_text(encoding="utf-8",errors="ignore").splitlines():
    line=line.strip()
    if not line or line[0] in "#": continue
    if not (line[0].isdigit() or line[0]=="-"): continue
    c=line.split(",")
    if len(c)<43: continue
    def f(i):
        try: return float(c[i])
        except: return float("nan")
    rows.append(dict(ms=f(5),phase=f(1),st=f(2),step=f(3),reach=f(4),mode=f(0),yaw=f(16),yawu=f(17),ox=f(32),oy=f(33),xt=f(21),yt=f(22),yawt=f(23),ex=f(24),ey=f(25),eyaw=f(26),vt=f(19),va=f(20),odo=f(34),elat=f(39)))

# yaw divergence over time
print("==== yaw chassis vs imu at key points ====")
for r in rows:
    if int(r["reach"])==1 or (int(r["step"])%2==1 and abs(r["eyaw"])<2.5 and abs(r["eyaw"])>0.01 and int(r["mode"])==1):
        pass
# sample every step end
prev=None
for r in rows:
    key=(int(r["st"]),int(r["step"]),int(r["phase"]),int(r["reach"]))
    if key!=prev and int(r["reach"])==1:
        print(f"t={r['ms']:.0f} step={int(r['step'])} yaw={r['yaw']:.2f} yawu={r['yawu']:.2f} diff={r['yaw']-r['yawu']:.2f} ox={r['ox']:.3f} oy={r['oy']:.3f}")
    prev=key

# overshoot after each reach before next step
print("\n==== overshoot after reach (within 350ms) ====")
i=0
while i < len(rows):
    r=rows[i]
    if int(r["reach"])==1 and int(r["st"])==1:
        t0=r["ms"]; ox0=r["ox"]; oy0=r["oy"]; yaw0=r["yaw"]; step=int(r["step"]); phase=int(r["phase"])
        j=i
        best=r
        while j < len(rows) and rows[j]["ms"]-t0 <= 350:
            best=rows[j]
            if int(rows[j]["step"])!=step: break
            j+=1
        dx=best["ox"]-ox0; dy=best["oy"]-oy0; dyaw=best["yaw"]-yaw0
        print(f"step{step} ph{phase} reach@({ox0:.3f},{oy0:.3f},{yaw0:.1f}) -> +350ms ({best['ox']:.3f},{best['oy']:.3f},{best['yaw']:.1f}) dxy={(dx*dx+dy*dy)**0.5:.4f} dyaw={dyaw:.2f}")
        i=j
        continue
    i+=1

# side3 slow reason: speed profile
print("\n==== side move durations and peak |elat| ====")
for step in [0,2,4,6]:
    seg=[r for r in rows if int(r["st"])==1 and int(r["step"])==step and int(r["phase"])==1]
    if not seg: continue
    elat=max(abs(r["elat"]) for r in seg)
    vt=max(abs(r["vt"]) for r in seg)
    print(f"step{step}: dt={seg[-1]['ms']-seg[0]['ms']:.0f} max|elat|={elat:.4f} max|vt_raw|={vt:.3f} start_elat={seg[0]['elat']:.4f} end=({seg[-1]['ox']:.3f},{seg[-1]['oy']:.3f})")
