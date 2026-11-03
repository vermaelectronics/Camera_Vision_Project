import sys, numpy as np, study as S
algos = [
 ("STAP ideal",    lambda c,t: S.stap(c,t,1,0)),
 ("STAP built",    lambda c,t: S.stap(c,t,75,15)),
 ("SFAP1024/512 l.94", lambda c,t: S.sfap(c,t,1024,512,0.9375,0,1.0,True)),
 ("SFAP1024/512 l.97", lambda c,t: S.sfap(c,t,1024,512,0.96875,0,1.0,True)),
 ("SFAP1024/256 l.98 hann", lambda c,t: S.sfap(c,t,1024,256,0.98,0)),
]
cases=[]
for jn in (30,40,50,60): cases += [("barrage",jn,False),("barrage",jn,True)]
for jn in (40,60): cases += [("cw",jn,False),("cw",jn,True)]
for p in (100,1000): cases += [(f"chirp{p}",40,False),(f"chirp{p}",40,True)]
cases += [("pulse",40,False),("pulse",50,True)]
print("cell: steady / mean-after-onset / worst / t<3dB(us)   algos:",[a[0] for a in algos],flush=True)
for kind,jn,mp in cases:
    comps,tot=S.scene(kind,jn,mp); row=[]
    for n,fn in algos:
        st,av,wo,t3=S.summarize(S.sinr_loss(fn(comps,tot),comps))
        row.append(f"{st:5.1f}/{av:5.1f}/{wo:5.1f}/{('inf' if t3==np.inf else int(t3)):>4}")
    print(f"{kind:9}{jn:3} {'mp' if mp else '  '} | "+" | ".join(row),flush=True)
