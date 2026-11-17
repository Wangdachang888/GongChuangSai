from pathlib import Path
rows=[]
for line in Path(r"E:\LOG1\log1.csv").read_text(encoding="utf-8",errors="ignore").splitlines():
    if not line or not (line[0].isdigit() or line[0]=='-'):
        continue
    c=line.split(',')
    if len(c)<43: continue
    if int(float(c[2]))==1 and int(float(c[3]))==4 and int(float(c[1]))==1:
        rows.append((float(c[5]), float(c[16]), float(c[23]), float(c[26]), float(c[14]), float(c[28]), float(c[29]), float(c[30]), float(c[31])))
# find sign flip of yaw
prev=None
for r in rows:
    if prev is not None and prev[1]*r[1] < 0 and abs(prev[1])>90 and abs(r[1])>90:
        print(f"YAW SIGN CROSS t={prev[0]:.0f}->{r[0]:.0f} yaw {prev[1]:.2f}->{r[1]:.2f} yawt={r[2]:.2f} eyaw={r[3]:.2f} w={r[4]:.3f} wheels={r[5]:.0f}/{r[6]:.0f}/{r[7]:.0f}/{r[8]:.0f}")
    # also print when |raw unwrap err| > 90
    raw=r[2]-r[1]
    if abs(raw)>90:
        print(f"UNWRAP BIG t={r[0]:.0f} yaw={r[1]:.2f} yawt={r[2]:.2f} raw={raw:.2f} eyaw={r[3]:.2f} w={r[4]:.3f}")
        break
    prev=r
# show first time w saturates
for r in rows:
    if abs(r[4])>3:
        raw=r[2]-r[1]
        print(f"FIRST SAT t={r[0]:.0f} yaw={r[1]:.2f} yawt={r[2]:.2f} raw_sub={raw:.2f} eyaw={r[3]:.2f} w={r[4]:.3f}")
        break
print('samples around cross:')
for r in rows:
    if 47000 <= r[0] <= 49000 or 39800 <= r[0] <= 40400:
        raw=r[2]-r[1]
        if int(r[0])%500 < 60:
            print(f"t={r[0]:.0f} yaw={r[1]:.2f} yawt={r[2]:.2f} raw={raw:.2f} eyaw={r[3]:.2f} w={r[4]:.3f} L/R ~ {(r[5]+r[7])/2:.0f}/{(r[6]+r[8])/2:.0f}")
