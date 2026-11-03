import numpy as np, subprocess
rng=np.random.default_rng(7); N=1024; NF=4
frames=[np.round(rng.standard_normal(N)*300+1j*rng.standard_normal(N)*300) for _ in range(NF)]
stream=[]
for x in frames: stream+= [(int(v.real),int(v.imag)) for v in x]
stream += [(0,0)]*(2*N)
with open("fft_in.hex","w") as fh:
    for re,im in stream: fh.write("%013x\n"%(((im&0x1FFFFFF)<<25)|(re&0x1FFFFFF)))
# tb handles NF*2048 samples: pad stream to that
open("fft_in.hex","a").write("".join("%013x\n"%0 for _ in range(NF*2048-len(stream))))
subprocess.run(["iverilog","-g2005","-Ptb_fft.SC=10'b1000000000","-o","tb_fft","tb_fft.v","../../../Source/HDL/sfap_fft.v","../../../Source/HDL/sfap_twiddle.v"],check=True,capture_output=True)
subprocess.run(["vvp","tb_fft"],check=True,capture_output=True)
rows=np.array([[int(t) if t.lstrip("-").isdigit() else 0 for t in ln.split()] for ln in open("fft_out.txt")],dtype=np.int64)
out=rows[:,1]+1j*rows[:,2]
br=np.array([int(format(k,'010b')[::-1],2) for k in range(N)])
for f,x in enumerate(frames):
    exp=(np.fft.fft(x)/2)[br]
    off=f*N+1068
    seg=out[off:off+N]
    print("frame",f,"err",round(np.sqrt(np.mean(abs(seg-exp)**2)),1),"rms",round(np.sqrt(np.mean(abs(exp)**2)),1))
