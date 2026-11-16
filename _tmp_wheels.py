from pathlib import Path
p=Path(r"E:\LOG1\log1.csv")
rows=[]
for line in p.read_text(encoding="utf-8",errors="ignore").splitlines():
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
    # mapping from bsp_ulog.c
    rows.append({
        "ms":f(5),"phase":int(f(1)),"st":int(f(2)),"step":int(f(3)),"reach":int(f(4)),"mode":int(f(0)),
        "yaw":f(16),"yawu":f(17),"vt":f(19),"va":f(20),
        "xt":f(21),"yt":f(22),"yawt":f(23),"ex":f(24),"ey":f(25),"eyaw":f(26),
        "lf_t":f(27),"lf":f(28),"rf":f(29),"lb":f(30),"rb":f(31),
        "ox":f(32),"oy":f(33),"odo":f(34),"elat":f(39),"herr":f(42),
        "w_tgt":f(14),"w_act":f(15)
    })

def span(vals):
    return max(vals)-min(vals)

print("==== per MOVE step: wheel spread ====")
for step in [0,2,4,6]:
    seg=[r for r in rows if r["st"]==1 and r["step"]==step and r["phase"]==1]
    if not seg: continue
    spreads=[]; asymmet=[]
    for r in seg:
        ws=[r["lf"],r["rf"],r["lb"],r["rb"]]
        spreads.append(span(ws))
        # left vs right average
        left=0.5*(r["lf"]+r["lb"]); right=0.5*(r["rf"]+r["rb"])
        asymmet.append(left-right)
    # find peak spread sample
    k=max(range(len(seg)), key=lambda i: span([seg[i]["lf"],seg[i]["rf"],seg[i]["lb"],seg[i]["rb"]]))
    r=seg[k]
    print(f"step{step}: n={len(seg)} dt={seg[-1]['ms']-seg[0]['ms']:.0f} max_spread={max(spreads):.1f} avg_spread={sum(spreads)/len(spreads):.1f} max|L-R|={max(abs(x) for x in asymmet):.1f}")
    print(f"  peak@ t={r['ms']:.0f} lf={r['lf']:.1f} rf={r['rf']:.1f} lb={r['lb']:.1f} rb={r['rb']:.1f} elat={r['elat']:.4f} eyaw={r['eyaw']:.2f} yaw={r['yaw']:.2f} ex={r['ex']:.3f} ey={r['ey']:.3f} w_tgt={r['w_tgt']:.3f}")

print("\n==== step4 (slow side) timeline every ~0.5s ====")
seg=[r for r in rows if r["st"]==1 and r["step"]==4 and r["phase"]==1]
last=None
for r in seg:
    if last is None or r["ms"]-last>=500:
        ws=[r["lf"],r["rf"],r["lb"],r["rb"]]
        left=0.5*(r["lf"]+r["lb"]); right=0.5*(r["rf"]+r["rb"])
        print(f"t={r['ms']:.0f} pos=({r['ox']:.3f},{r['oy']:.3f}) elat={r['elat']:+.4f} eyaw={r['eyaw']:+.2f} L={left:.1f} R={right:.1f} dLR={left-right:+.1f} lf/rf/lb/rb={r['lf']:.1f}/{r['rf']:.1f}/{r['lb']:.1f}/{r['rb']:.1f} spread={span(ws):.1f} w_tgt={r['w_tgt']:.3f}")
        last=r["ms"]

print("\n==== compare first 1s of each side ====")
for step in [0,2,4,6]:
    seg=[r for r in rows if r["st"]==1 and r["step"]==step and r["phase"]==1]
    if not seg: continue
    t0=seg[0]["ms"]
    early=[r for r in seg if r["ms"]-t0<=1000]
    spreads=[span([r["lf"],r["rf"],r["lb"],r["rb"]]) for r in early]
    dlr=[0.5*(r["lf"]+r["lb"])-0.5*(r["rf"]+r["rb"]) for r in early]
    print(f"step{step} first1s: start_elat={seg[0]['elat']:.4f} start_eyaw={seg[0]['eyaw']:.2f} avg_spread={sum(spreads)/len(spreads):.1f} avg_dLR={sum(dlr)/len(dlr):.1f} max_spread={max(spreads):.1f}")
