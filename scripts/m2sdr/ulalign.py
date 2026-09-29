import numpy as np, re, sys
SLOT=11520
con=open('/home/dmd/ocudu/build/captures/gnb_console.log').read()
tx_ts={int(m.group(2)):int(m.group(1)) for m in re.finditer(r'TX dump: wrote \d+ samples from ts=(\d+) to \S*g_tx\.cs16\.(\d+)',con)}
rx_ts={int(m.group(2)):int(m.group(1)) for m in re.finditer(r'RX dump: wrote \d+ samples starting at ts=(\d+) to \S*g_rx\.cs16\.(\d+)',con)}
print("TX dump ts mod SLOT:", {k:v%SLOT for k,v in tx_ts.items()})
def load(p):
    d=np.fromfile(p,dtype=np.int16).astype(np.float32); return d[0::2]+1j*d[1::2]
# TX grid phase: TX dump content — find slot boundary phase by maximizing the contrast of per-slot power over candidate phases
def tx_phase(x):
    best=None
    for ph in range(0,SLOT,64):
        n=(len(x)-ph)//SLOT; p=np.array([np.mean(np.abs(x[ph+i*SLOT:ph+(i+1)*SLOT])**2) for i in range(n)])
        # boundary quality: many slots are exactly zero (UL/special/empty) -> count of near-zero slots is maximal when aligned
        q=np.sum(p<1e-3*p.max()); 
        if best is None or q>best[1]: best=(ph,q)
    return best
for g in sorted(k for k in rx_ts if k>0):
    tx=load(f'/home/dmd/ocudu/build/captures/g_tx.cs16.{g}'); rx=load(f'/home/dmd/ocudu/build/captures/g_rx.cs16.{g}')
    ph,q=tx_phase(tx); grid0=tx_ts[g]+ph     # a TX slot boundary timestamp (shared clock)
    # UL bursts in RX: 64-sample power, threshold
    b=64; nb=len(rx)//b; p=np.mean(np.abs(rx[:nb*b].reshape(nb,b))**2,axis=1); noise=np.median(p); on=p>noise*10
    starts=[i*b for i in range(1,nb) if on[i] and not on[i-1]]; ends=[i*b for i in range(1,nb) if (not on[i]) and on[i-1]]
    print(f"\n--- gen {g}: TX grid boundary ts={grid0} (phase {ph}, zero-slots {q}); RX dump ts={rx_ts[g]} ---")
    for s in starts:
        e=min([x for x in ends if x>s], default=None)
        if e is None or e-s<3000: continue
        # refine start to sample resolution
        seg=np.abs(rx[max(0,s-128):s+128])**2; k=np.argmax(seg>noise*10); s_ref=max(0,s-128)+k
        ts_burst=rx_ts[g]+s_ref; d=(ts_burst-grid0)%SLOT
        if d>SLOT//2: d-=SLOT
        print(f"  burst len={e-s_ref:6d} smp ({(e-s_ref)/11520*14:4.1f} sym)  start offset vs TX slot grid = {d:+6d} samples = {d/23.04:+7.1f} us   (768-sample PRACH-symbol residual: {((d+384)%768)-384:+5d})")
