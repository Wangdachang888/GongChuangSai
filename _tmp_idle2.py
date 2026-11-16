from pathlib import Path
rows=[]
for line in Path(r"E:\LOG1\log1.csv").read_text(encoding="utf-8",errors="ignore").splitlines():
    line=line.strip()
    if not line or line.startswith("#") or not (line[0].isdigit() or line[0]=="-"):
        continue
    c=line.split(",")
    if len(c)<43: continue
    vals=[float(x) if x not in ("","nan") else float("nan") for x in c[:43]]
    rows.append(vals)

idle=[]
for r in rows:
    if int(r[1])==1: break
    idle.append(r)
print("idle samples",len(idle))
# for each channel, start/end/max abs during idle
print("ch  start    end     maxabs   (only if changes)")
for ch in range(43):
    xs=[r[ch] for r in idle]
    if max(xs)-min(xs) > 0.05 or abs(xs[-1]-xs[0])>0.05:
        print(f"{ch:2d} {xs[0]:+8.3f} {xs[-1]:+8.3f} {max(abs(x) for x in xs):8.3f}")

# also first 2s after move start yawu
move=[r for r in rows if int(r[1])==1][:5]
print("first move yaw/yawu:", [(r[5], r[16], r[17]) for r in move[:3]])
# whole-run when |yawu| first exceeds 2.5 and 3
for thr in (2.5,3.0,3.5):
    for r in rows:
        if abs(r[17])>=thr:
            print(f"|yawu|>={thr} first at t={r[5]:.0f} phase={int(r[1])} step={int(r[3])} yawu={r[17]:+.2f} yaw={r[16]:+.2f}")
            break
