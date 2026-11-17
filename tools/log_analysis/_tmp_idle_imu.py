from pathlib import Path
p=Path(r"E:\LOG1\log1.csv")
rows=[]
for line in p.read_text(encoding="utf-8", errors="ignore").splitlines():
    line=line.strip()
    if not line or line.startswith("#"):
        continue
    if not (line[0].isdigit() or line[0]=="-"):
        continue
    c=line.split(",")
    if len(c)<43:
        continue
    def f(i):
        try: return float(c[i])
        except: return float("nan")
    rows.append({
        "ms":f(5),"phase":int(f(1)),"st":int(f(2)),"step":int(f(3)),"mode":int(f(0)),"reach":int(f(4)),
        "yaw":f(16),"yawu":f(17),"yrate":f(18),
        "ox":f(32),"oy":f(33),"odo":f(34),
        "lf":f(28),"rf":f(29),"lb":f(30),"rb":f(31)
    })
print("file",p,"rows",len(rows),"ms",rows[0]["ms"],"..",rows[-1]["ms"])
# idle until first move phase
idle=[]
for r in rows:
    if r["phase"] in (1,2,4) and (abs(r["lf"])+abs(r["rf"])+abs(r["lb"])+abs(r["rb"])>5 or abs(r["ox"])+abs(r["oy"])>0.01):
        # keep collecting pure phase0 first
        pass
    if r["phase"]==0:
        idle.append(r)
    elif r["phase"]==1:
        break
    else:
        idle.append(r)

# more robust: from start until first sample with phase==1
idle=[]
for r in rows:
    if r["phase"]==1:
        break
    idle.append(r)
print(f"idle n={len(idle)} t={idle[0]['ms']:.0f}..{idle[-1]['ms']:.0f} dt={(idle[-1]['ms']-idle[0]['ms'])/1000:.2f}s")
print(f"start yaw={idle[0]['yaw']:+.3f} yawu={idle[0]['yawu']:+.3f} yrate={idle[0]['yrate']:+.4f}")
print(f"end   yaw={idle[-1]['yaw']:+.3f} yawu={idle[-1]['yawu']:+.3f} yrate={idle[-1]['yrate']:+.4f}")
dy=idle[-1]['yawu']-idle[0]['yawu']
dt=(idle[-1]['ms']-idle[0]['ms'])/1000
print(f"yawu drift={dy:+.3f} deg over {dt:.2f}s => {dy/dt:+.4f} deg/s")
ys=[r['yrate'] for r in idle]
print(f"yrate avg={sum(ys)/len(ys):+.4f} min={min(ys):+.4f} max={max(ys):+.4f}")
print("\n==== every 1s ====")
last=None
for r in idle:
    if last is None or r['ms']-last>=1000:
        print(f"t={r['ms']:7.0f} yaw={r['yaw']:+7.3f} yawu={r['yawu']:+7.3f} yrate={r['yrate']:+7.3f} wheels={r['lf']+r['rf']+r['lb']+r['rb']:.2f} odo={r['odo']:.3f}")
        last=r['ms']
# find when yawu crosses 1,2,3
for thr in (1.0,2.0,3.0,-1.0,-2.0,-3.0):
    hit=next((r for r in idle if (r['yawu']>=thr if thr>0 else r['yawu']<=thr)), None)
    if hit:
        print(f"yawu reaches {thr:+.1f} at t={hit['ms']:.0f} (after {(hit['ms']-idle[0]['ms'])/1000:.2f}s)")
# first move sample
move=next(r for r in rows if r['phase']==1)
print(f"\nfirst move t={move['ms']:.0f} yaw={move['yaw']:+.3f} yawu={move['yawu']:+.3f}")
