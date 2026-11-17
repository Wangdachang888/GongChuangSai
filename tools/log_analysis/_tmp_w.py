from pathlib import Path
p=Path(r"E:\LOG1\log1.csv")
# confirm w_tgt during step4 vs eyaw and yawt/yaw
for line in p.read_text(encoding="utf-8",errors="ignore").splitlines():
    if not line or not (line[0].isdigit() or line[0]=='-'):
        continue
    c=line.split(',')
    if len(c)<43: continue
    step=int(float(c[3])); phase=int(float(c[1])); st=int(float(c[2]))
    if st==1 and step==4 and phase==1:
        ms=float(c[5]); yaw=float(c[16]); yawt=float(c[23]); eyaw=float(c[26]); wt=float(c[14])
        # raw unwrapped error in deg
        raw = yawt - yaw
        print(f"t={ms:.0f} yaw={yaw:.2f} yawt={yawt:.2f} eyaw_log={eyaw:.2f} raw_sub={raw:.2f} w_tgt={wt:.3f}")
        break
# count how often w saturates on step4
n=0; sat=0
for line in p.read_text(encoding="utf-8",errors="ignore").splitlines():
    if not line or not (line[0].isdigit() or line[0]=='-'):
        continue
    c=line.split(',')
    if len(c)<43: continue
    if int(float(c[2]))==1 and int(float(c[3]))==4 and int(float(c[1]))==1:
        n+=1
        if abs(float(c[14]))>3.0: sat+=1
print(f"step4 samples={n} omega_sat(|w|>3)={sat} ({100*sat/n:.1f}%)")
