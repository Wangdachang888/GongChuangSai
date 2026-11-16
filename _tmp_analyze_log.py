from pathlib import Path
p=Path(r"E:\LOG1\log1.csv")
rows=[]
for i,line in enumerate(p.read_text(encoding="utf-8",errors="ignore").splitlines()):
    line=line.strip()
    if not line or line.startswith("#") or not (line[0].isdigit() or line[0]=="-"):
        continue
    cols=line.split(",")
    if len(cols)<43:
        continue
    def f(idx):
        try: return float(cols[idx])
        except: return float("nan")
    rows.append({
        "i":i+1,
        "mode":f(0),"phase":f(1),"sq_st":f(2),"sq_step":f(3),"reach":f(4),"ms":f(5),
        "yaw":f(16),"yawu":f(17),"yrate":f(18),"vt":f(19),"va":f(20),
        "xt":f(21),"yt":f(22),"yawt":f(23),"ex":f(24),"ey":f(25),"eyaw":f(26),
        "ox":f(32),"oy":f(33),"odo":f(34),"elat":f(39),"herr":f(42),
        "w_tgt":f(14),"w_act":f(15)
    })
print("rows",len(rows),"ms",rows[0]["ms"],"..",rows[-1]["ms"])
prev=None
print("==== state transitions ====")
for r in rows:
    key=(int(r["phase"]),int(r["sq_st"]),int(r["sq_step"]),int(r["mode"]),int(r["reach"]))
    if key!=prev:
        print(f"t={r['ms']:8.0f} phase={key[0]} st={key[1]} step={key[2]} mode={key[3]} reach={key[4]} ox={r['ox']:.4f} oy={r['oy']:.4f} yaw={r['yaw']:.2f} yawu={r['yawu']:.2f} tgt=({r['xt']:.3f},{r['yt']:.3f},{r['yawt']:.1f}) eyaw={r['eyaw']:.2f} elat={r['elat']:.4f} odo={r['odo']:.3f}")
        prev=key
print("==== end ====")
r=rows[-1]
print(f"END ox={r['ox']:.4f} oy={r['oy']:.4f} yaw={r['yaw']:.2f} yawu={r['yawu']:.2f} odo={r['odo']:.3f}")

# segment by step+phase while running
seg=[]; cur=None
for r in rows:
    k=(int(r["sq_st"]),int(r["sq_step"]),int(r["phase"]))
    if cur is None or k!=cur["k"]:
        if cur: seg.append(cur)
        cur={"k":k,"a":r,"b":r,"n":0,"elat":[],"herr":[],"eyaw":[],"va":[],"vt":[]}
    cur["b"]=r; cur["n"]+=1
    cur["elat"].append(abs(r["elat"])); cur["herr"].append(abs(r["herr"])); cur["eyaw"].append(abs(r["eyaw"]))
    cur["va"].append(abs(r["va"])); cur["vt"].append(abs(r["vt"]))
if cur: seg.append(cur)

def avgmax(xs):
    xs=[x for x in xs if x==x]
    if not xs: return 0,0
    return sum(xs)/len(xs), max(xs)

print("\n==== segments ====")
for s in seg:
    a,b=s["a"],s["b"]
    ea,em=avgmax(s["elat"]); ha,hm=avgmax(s["herr"]); ya,ym=avgmax(s["eyaw"]); va,vm=avgmax(s["va"]); vt,_=avgmax(s["vt"])
    dx=b["ox"]-a["ox"]; dy=b["oy"]-a["oy"]; dist=(dx*dx+dy*dy)**0.5
    print(f"st{s['k'][0]} step{s['k'][1]} ph{s['k'][2]} n={s['n']:4d} t={a['ms']:.0f}-{b['ms']:.0f} dt={b['ms']-a['ms']:.0f} dxy={dist:.3f} odoΔ={b['odo']-a['odo']:.3f} end=({b['ox']:.3f},{b['oy']:.3f},{b['yaw']:.1f}/{b['yawu']:.1f}) tgt=({b['xt']:.3f},{b['yt']:.3f},{b['yawt']:.1f}) |elat|avg/max={ea:.4f}/{em:.4f} |herr|={ha:.2f}/{hm:.2f} end_eyaw={b['eyaw']:.2f} vavg={va:.3f}")

# estimate square corners at end of each MOVE phase (phase mapped in ulog)
print("\n==== move/turn endpoints ====")
for s in seg:
    if s["k"][0]!=1: continue
    b=s["b"]
    print(f"step{s['k'][1]} ph{s['k'][2]} -> ({b['ox']:.4f},{b['oy']:.4f}) yaw={b['yaw']:.2f} yawu={b['yawu']:.2f} err_xy=({b['ex']:.4f},{b['ey']:.4f}) eyaw={b['eyaw']:.2f}")

# closed-loop closure error relative to start of motion
motion=[r for r in rows if int(r["sq_st"])==1 or int(r["phase"]) in (1,2,4)]
if motion:
    s0=motion[0]; s1=rows[-1]
    print("\n==== closure ====")
    print(f"start ({s0['ox']:.4f},{s0['oy']:.4f},{s0['yaw']:.2f})")
    print(f"end   ({s1['ox']:.4f},{s1['oy']:.4f},{s1['yaw']:.2f})")
    print(f"dx={s1['ox']-s0['ox']:.4f} dy={s1['oy']-s0['oy']:.4f} dyaw={s1['yaw']-s0['yaw']:.2f}")
    print(f"final odo={s1['odo']:.4f} expected perimeter~8.0")
