import numpy as np, sys, re, subprocess
fs=23.04e6; N=768; PCI=101; OFF=int(sys.argv[5]) if len(sys.argv)>5 else -176
log=f'TX dump: wrote 0 samples from ts={sys.argv[3]} RX dump: wrote 0 samples starting at ts={sys.argv[4]}' if len(sys.argv)>4 else subprocess.run(['sudo','grep','-a','-E','TX dump: wrote|RX dump: wrote','/tmp/gnb.log'],capture_output=True,text=True).stdout
txts=int(re.search(r'TX dump: wrote \d+ samples from ts=(\d+)',log).group(1)); rxts=int(re.search(r'RX dump: wrote \d+ samples starting at ts=(\d+)',log).group(1))
def pss(nid2):
    xs=[0,1,1,0,1,1,1]
    for i in range(120): xs.append((xs[i+4]+xs[i])%2)
    return np.array([1-2*xs[(n+43*nid2)%127] for n in range(127)],float)
X=np.zeros(N,complex); X[[(OFF+56+n-120)%N for n in range(127)]]=pss(PCI%3); t=np.fft.ifft(X); t/=np.linalg.norm(t)
def first(f,div):
    x=np.fromfile(f,dtype=np.int16).astype(np.complex128); x=(x[0::2]+1j*x[1::2])/div; L=len(x)
    c=np.abs(np.fft.ifft(np.fft.fft(x)*np.conj(np.fft.fft(t,L))))[:L-N]
    E=np.sqrt(np.convolve(np.abs(x)**2,np.ones(N),'valid'))[:L-N]; nc=c/(E+1e-6); nc[E<0.2*E.max()]=0
    k=int(np.argmax(nc)); return k, float(nc[k])
ktx,ctx=first(sys.argv[1],1); krx,crx=first(sys.argv[2],16)
d=((rxts+krx)-(txts+ktx))%230400
if d>115200: d-=230400
print(f"TX PSS ts%frame={(txts+ktx)%230400} (corr {ctx:.2f})  RX PSS ts%frame={(rxts+krx)%230400} (corr {crx:.2f})  ->  RX-minus-TX offset {d} samples = {d/fs*1e6:.1f} us")
