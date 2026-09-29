import numpy as np, sys
FS=23.04e6; SLOT=11520; PER=10*SLOT; NFFT=768; NRB=51
def load(p):
    d=np.fromfile(p,dtype=np.int16).astype(np.float32); return d[0::2]+1j*d[1::2]
def slotpow(x,off,n):
    return np.array([10*np.log10(np.mean(np.abs(x[off+i*SLOT:off+(i+1)*SLOT])**2)+1e-9) for i in range(n)])
def find_phase(x):
    # rising edge of the DL leakage every 5 ms -> slot-0/10 boundary. coarse 256-sample bins.
    b=256; nb=len(x)//b; p=10*np.log10(np.mean(np.abs(x[:nb*b].reshape(nb,b))**2,axis=1)+1e-9)
    per=PER//b; fold=np.zeros(per)
    for i in range(per): fold[i]=np.median(p[i::per])
    d=np.roll(fold,-1)-fold; k=int(np.argmax(d)); return ((k+1)*b)%PER, fold
def rb_spectrum(seg):
    n=len(seg)//NFFT; S=np.abs(np.fft.fftshift(np.fft.fft(seg[:n*NFFT].reshape(n,NFFT),axis=1),axes=1))**2
    psd=S.mean(0); sc=np.arange(NFFT)-NFFT//2   # subcarrier index rel. carrier centre
    rb=np.floor((sc+NRB*6)/12).astype(int); out=np.full(NRB,-200.0)
    for r in range(NRB):
        m=(rb==r); 
        if m.any(): out[r]=10*np.log10(psd[m].mean()+1e-9)
    return out
def burst_edges(seg,thr_db):
    b=64; nb=len(seg)//b; p=10*np.log10(np.mean(np.abs(seg[:nb*b].reshape(nb,b))**2,axis=1)+1e-9)
    on=np.where(p>thr_db)[0]
    return (on[0]*b, (on[-1]+1)*b) if len(on) else (None,None)
for path in sys.argv[1:]:
    x=load(path); ph,fold=find_phase(x); n=(len(x)-ph)//SLOT
    sp=slotpow(x,ph,n); ul_idx=[i for i in range(n) if i%10 in (7,8,9)]
    noise=np.median(sp[ul_idx]); dl=np.median([sp[i] for i in range(n) if i%10<6])
    print(f"\n=== {path}: phase={ph} samples, slots={n}, DL-leak={dl:.1f} dB, UL-noise={noise:.1f} dB ===")
    line=''.join('D' if i%10<6 else ('s' if i%10==6 else ('#' if sp[i]>noise+6 else '.')) for i in range(n))
    print("pattern (D=DL,s=special,.=quiet UL,#=UL burst), one char per slot, '|' every 10:")
    print('|'.join(line[i:i+10] for i in range(0,n,10)))
    for i in range(n):
        if i%10 in (7,8,9) and sp[i]>noise+6:
            seg=x[ph+i*SLOT:ph+(i+1)*SLOT]; rbs=rb_spectrum(seg); top=np.argsort(rbs)[::-1]
            act=[int(r) for r in range(NRB) if rbs[r]>np.median(rbs)+8]
            a,bx=burst_edges(seg, noise+6)
            print(f"  slot#{i:3d} (idx {i%10}) pow={sp[i]:.1f}dB (+{sp[i]-noise:.1f}) RBs_active={act[:16]}{'...' if len(act)>16 else ''}  burst_samples=[{a},{bx}) = symbols[{a/822.9 if a is not None else -1:.1f},{bx/822.9 if bx is not None else -1:.1f})")
