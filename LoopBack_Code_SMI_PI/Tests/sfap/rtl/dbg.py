import sys, subprocess, numpy as np
load = sys.argv[1] if len(sys.argv)>1 else "2000000000"
subprocess.run(["iverilog","-g2005",f"-Ptb_core.NS=20000",f"-Ptb_core.LOAD={load}","-o","tb_core","tb_core.v"]+
  ["../../../Source/HDL/"+f for f in ("sfap_core.v","sfap_fft.v","sfap_bin.v","sfap_twiddle.v","sfap_recip_rom.v","sfap_win_rom.v")],check=True,capture_output=True)
subprocess.run(["vvp","tb_core"],check=True,capture_output=True)
rows=np.array([[int(t) for t in l.split()] for l in open("core_out.txt")])
y=rows[:,1]+1j*rows[:,2]; x=np.load("core_x1.npy")
print("out rms",np.sqrt(np.mean(abs(y[3000:])**2)),"in rms",np.sqrt(np.mean(abs(x)**2)))
best=[]
for d in range(0,4000):
    n0=2000; n1=4000
    if n1+d>len(y): break
    best.append((abs(np.vdot(x[n0:n1],y[n0+d:n1+d]))/ (np.linalg.norm(x[n0:n1])*np.linalg.norm(y[n0+d:n1+d])+1e-9), d))
best.sort(reverse=True); print(best[:5])
