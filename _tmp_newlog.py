from pathlib import Path
p=Path(r"E:\LOG1\log1.000101.000001.csv")
print("exists", p.exists(), "size", p.stat().st_size if p.exists() else None)
rows=[]
meta=[]
for i,line in enumerate(p.read_text(encoding="utf-8", errors="ignore").splitlines()):
    s=line.strip()
    if not s: continue
    if s.startswith("#"):
        meta.append((i+1,s)); continue
    if not (s[0].isdigit() or s[0]=="-"):
        meta.append((i+1,s)); continue
    c=s.split(",")
    if len(c)<43:
        meta.append((i+1,f"short_cols={len(c)}")); continue
    def f(k):
        try: return float(c[k])
        except: return float("nan")
    rows.append({
        "ms":f(5),"phase":int(f(1)),"st":int(f(2)),"step":int(f(3)),"mode":int(f(0)),"reach":int(f(4)),
        "yaw":f(16),"yawu":f(17),"yrate":f(18),"vt":f(19),"va":f(20),
        "xt":f(21),"yt":f(22),"yawt":f(23),"ex":f(24),"ey":f(25),"eyaw":f(26),
        "lf":f(28),"rf":f(29),"lb":f(30),"rb":f(31),
        "ox":f(32),"oy":f(33),"odo":f(34),"elat":f(39),"herr":f(42),"wt":f(14)
    })
print("meta", meta[:10])
print("rows", len(rows), "ms", rows[0]["ms"], "..", rows[-1]["ms"])
# idle
idle=[]
for r in rows:
    if r["phase"]==1: break
    idle.append(r)
print("\n==== IDLE drift ====")
print(f"n={len(idle)} dt={(idle[-1]['ms']-idle[0]['ms'])/1000:.2f}s")
print(f"yaw  {idle[0]['yaw']:+.3f} -> {idle[-1]['yaw']:+.3f}")
print(f"yawu {idle[0]['yawu']:+.3f} -> {idle[-1]['yawu']:+.3f} peak={max(abs(r['yawu']) for r in idle):.3f}")
ys=[r["yrate"] for r in idle]
print(f"yrate avg={sum(ys)/len(ys):+.4f} maxabs={max(abs(y) for y in ys):.4f}")
dy=idle[-1]["yawu"]-idle[0]["yawu"]; dt=(idle[-1]["ms"]-idle[0]["ms"])/1000
print(f"yawu rate={dy/dt:+.4f} deg/s")
last=None
print("idle every 2s:")
for r in idle:
    if last is None or r["ms"]-last>=2000:
        print(f"  t={r['ms']:.0f} yaw={r['yaw']:+.3f} yawu={r['yawu']:+.3f} yrate={r['yrate']:+.3f}")
        last=r["ms"]

# transitions
print("\n==== transitions ====")
prev=None
for r in rows:
    key=(r["phase"],r["st"],r["step"],r["mode"],r["reach"])
    if key!=prev:
        print(f"t={r['ms']:8.0f} ph={r['phase']} st={r['st']} step={r['step']} mode={r['mode']} reach={r['reach']} ox={r['ox']:.4f} oy={r['oy']:.4f} yaw={r['yaw']:.2f} yawu={r['yawu']:.2f} eyaw={r['eyaw']:.2f} elat={r['elat']:.4f} odo={r['odo']:.3f}")
        prev=key

# segments
seg=[]; cur=None
for r in rows:
    k=(r["st"],r["step"],r["phase"])
    if cur is None or k!=cur["k"]:
        if cur: seg.append(cur)
        cur={"k":k,"a":r,"b":r,"n":0,"spread":[],"dlr":[],"wt":[]}
    cur["b"]=r; cur["n"]+=1
    ws=[r["lf"],r["rf"],r["lb"],r["rb"]]
    cur["spread"].append(max(ws)-min(ws))
    cur["dlr"].append(0.5*(r["lf"]+r["lb"])-0.5*(r["rf"]+r["rb"]))
    cur["wt"].append(abs(r["wt"]))
if cur: seg.append(cur)
print("\n==== segments ====")
for s in seg:
    a,b=s["a"],s["b"]
    dx=b["ox"]-a["ox"]; dy=b["oy"]-a["oy"]; dist=(dx*dx+dy*dy)**0.5
    sat=sum(1 for w in s["wt"] if w>3.0)
    print(f"st{s['k'][0]} step{s['k'][1]} ph{s['k'][2]} n={s['n']:4d} dt={b['ms']-a['ms']:.0f} dxy={dist:.3f} odoΔ={b['odo']-a['odo']:.3f} end=({b['ox']:.3f},{b['oy']:.3f},{b['yaw']:.1f}/{b['yawu']:.1f}) maxSpread={max(s['spread']) if s['spread'] else 0:.1f} max|L-R|={max(abs(x) for x in s['dlr']) if s['dlr'] else 0:.1f} omegaSat={sat}/{s['n']}")

r=rows[-1]
print(f"\n==== END ====")
print(f"ox={r['ox']:.4f} oy={r['oy']:.4f} yaw={r['yaw']:.2f} yawu={r['yawu']:.2f} diff={r['yaw']-r['yawu']:.2f} odo={r['odo']:.3f}")
# yaw divergence at key points
print("\n==== yaw vs yawu at reach ====")
prev=None
for r in rows:
    key=(r["st"],r["step"],r["reach"])
    if r["reach"]==1 and key!=prev:
        print(f"t={r['ms']:.0f} step={r['step']} yaw={r['yaw']:.2f} yawu={r['yawu']:.2f} diff={r['yaw']-r['yawu']:.2f}")
    prev=key
# overshoot after reach
print("\n==== overshoot 350ms after reach ====")
i=0
while i < len(rows):
    r=rows[i]
    if r["reach"]==1 and r["st"]==1:
        t0=r["ms"]; ox0=r["ox"]; oy0=r["oy"]; step=r["step"]; phase=r["phase"]
        best=r; j=i
        while j < len(rows) and rows[j]["ms"]-t0<=350:
            best=rows[j]
            if rows[j]["step"]!=step: break
            j+=1
        d=((best["ox"]-ox0)**2+(best["oy"]-oy0)**2)**0.5
        print(f"step{step} ph{phase} ({ox0:.3f},{oy0:.3f}) -> ({best['ox']:.3f},{best['oy']:.3f}) dxy={d:.4f}")
        i=j; continue
    i+=1
