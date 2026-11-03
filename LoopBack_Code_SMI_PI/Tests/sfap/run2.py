import sys, numpy as np, study as S
algos = [
 ("STAP-16 ideal", lambda c,t: S.stap(c,t,1,0)),
 ("K512 s8 l.93",  lambda c,t: S.sfap(c,t,512,128,0.93,8)),
 ("K1024 s16 l.85",lambda c,t: S.sfap(c,t,1024,256,0.85,16)),
 ("K1024 s32 l.85",lambda c,t: S.sfap(c,t,1024,256,0.85,32)),
]
cases = [("barrage",50,True),("barrage",60,True),("cw",60,True),("chirp100",40,True),("pulse",50,True)]
print("cell: steady / mean-after-onset / worst / t<3dB us")
for kind,jn,mp in cases:
    comps,tot = S.scene(kind,jn,mp)
    row=[]
    for n,fn in algos:
        st,av,wo,t3 = S.summarize(S.sinr_loss(fn(comps,tot),comps))
        row.append(f"{n}: {st:5.1f}/{av:5.1f}/{wo:5.1f}/{('inf' if t3==np.inf else int(t3))}")
    print(kind,jn,"mp" if mp else "", " | ".join(row), flush=True)
