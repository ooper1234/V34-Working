import wave, numpy as np
with wave.open("/tmp/opencode/v90cap/line-02393210-0.wav",'rb') as w:
    fs=w.getframerate(); raw=np.frombuffer(w.readframes(w.getnframes()),dtype='<i2')
a=raw.reshape(-1,2).astype(np.float64)
rx=a[:,0]/32768.0; tx=a[:,1]/32768.0
LO,HI=12.6,14.5
X=rx[int(LO*fs):int(HI*fs)]; R=tx[int(LO*fs):int(HI*fs)]
n=min(len(X),len(R)); X=X[:n]; R=R[:n]
TAPS=512; DELAY=1320; C=TAPS//2; MU=0.5

def nlms(X,R,delay,taps,mu=0.5,leak=0.99995):
    c=taps//2; d0=delay-c
    w=np.zeros(taps)
    idx=np.arange(len(X))[:,None]-d0-np.arange(taps)[None,:]
    ok=(idx>=0)&(idx<len(R))
    idxc=np.where(ok,idx,0)
    rows=np.where(ok,R[idxc],0.0)
    for i in range(len(X)):
        row=rows[i]
        yhat=row@w
        norm=row@row
        if norm>1e-4:
            g=mu*(X[i]-yhat)/norm
            w=w*leak+g*row
    return w
def ls(X,R,delay,taps):
    c=taps//2; nfft=1
    while nfft < 2*delay: nfft*=2
    win=np.hanning(nfft+2)[1:-1]; step=nfft//4
    srr=np.zeros(nfft//2+1); sxy=np.zeros(nfft//2+1,complex); nb=0
    for o in range(0, max(1,len(X)-nfft), step):
        xr=X[o:o+nfft]; rr=R[o:o+nfft]
        if len(xr)<nfft: break
        F=np.fft.rfft(xr*win); G=np.fft.rfft(rr*win)
        srr+=np.abs(G)**2; sxy+=F*G.conj(); nb+=1
    pk=srr.max()
    H=np.where(srr>=pk*1e-4, sxy/(srr+pk*1e-6), 0)
    h=np.fft.irfft(H,nfft); p=int(np.argmax(np.abs(h)))
    w=np.zeros(taps)
    for i in range(nfft):
        k=i+p-delay+c
        if 0<=k<taps: w[k]=h[i]
    return w,p,nb,nfft
def pred(w,X,R,delay):
    c=len(w)//2; d0=delay-c
    idx=np.arange(len(X))[:,None]-d0-np.arange(len(w))[None,:]
    ok=(idx>=0)&(idx<len(R))
    return np.where(ok,R[np.where(ok,idx,0)],0.0)@w
def txpow(res,R,delay,taps,step=8):
    c=taps//2; best=0.0
    for lag in range(max(0,delay-c), min(delay+c, max(1,len(R)-256))):
        v=float(res[lag:].dot(R[:len(res)-lag])/(len(res)-lag))
        best=max(best, v*v)
    return best*len(res)
wG=nlms(X,R,DELAY,TAPS)
wL,peak,nb,nfft=ls(X,R,DELAY,TAPS)
pG=pred(wG,X,R,DELAY); pL=pred(wL,X,R,DELAY)
rN=X; rG=X-pG; rL=X-pL
P0,G,L=txpow(rN,R,DELAY,TAPS),txpow(rG,R,DELAY,TAPS),txpow(rL,R,DELAY,TAPS)
rms=lambda v: float(np.sqrt((v**2).mean()))
erle=lambda a,b: 10*np.log10(a/max(b,1e-30))
print(f"OFFLINE A/B on a real DIL capture (line-02393210, 12.6-14.5 s, {n} samples)")
print(f"  ECHO A/B: delay={DELAY} window={n}   LS: {nb} blocks of {nfft}, peak at {peak}\n")
print(f"    uncancelled   total RMS {rms(rN):.3e}   TX-correlated {P0:.3e}")
print(f"    gradient      total RMS {rms(rG):.3e}   TX-correlated {G:.3e}   ERLE_tx {erle(P0,G):5.1f} dB   norm {float(np.sqrt((wG**2).sum())):.4f}")
print(f"    block LS      total RMS {rms(rL):.3e}   TX-correlated {L:.3e}   ERLE_tx {erle(P0,L):5.1f} dB   norm {float(np.sqrt((wL**2).sum())):.4f}")
print(f"    selected={'LS' if L<G else 'gradient'}")
print("\n  old depth metric, same data (kept for the record, not used):")
for nm,p in (("gradient",pG),("LS",pL)):
    print(f"    {nm:9s} {10*np.log10((p**2).sum()/max(((X-p)**2).sum(),1e-30)):6.1f} dB")
