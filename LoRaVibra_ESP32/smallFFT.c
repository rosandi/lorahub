
#include <stdint.h>

#define NTAB 50
#define PI 3.141592653589793
#define PP 6.283185307179586
#define PH 1.570796326794897

#define AMAX 255

struct {float re; float im;} w, wm, t, u;  


unsigned char sintab[NTAB] = 
{0, 16, 31, 47, 63, 78,
 93, 108, 122, 136, 149, 162,
 174, 185, 196, 206, 215, 223,
 230, 237, 242, 246, 250, 252,
 254, 255, 254, 252, 250, 246,
 242, 237, 230, 223, 215, 206,
 196, 185, 174, 162, 149, 136,
 122, 108, 93, 78, 63, 47, 31, 16};

float rsin(float x) {
    int qu=1;
    while (x>PP) x-=PP;
    while (x<0) x+=PP;
    if(x>=PI) {x-=PI;qu=-1;}

    int xx=(int)(x/PP);
    x-=(PP*(float)xx);
    x*=100.0/PP;
    xx=(int)x;
    float ofs=(x-(float)xx);
    float b=xx<(NTAB-1)?(float)sintab[xx+1]:(float)sintab[0];
    b=qu*(sintab[xx]+(b-sintab[xx])*ofs);
    return b/AMAX;
}

float rcos(float x) {
    return rsin(x+PH);
}

int16_t fft(int16_t *val, int16_t *ival, int log2N) {
  int i,j,k,s,m,m2,n;
  
  n=(1<<log2N);
  
    // bit reversal       
    for (i = 0; i < n; ++i) {
        k=0;
        s=i;       
        for (j = 0; j < log2N; j++) {
            k <<= 1;
            k |= (s & 1);
            s >>= 1;
        }        
        ival[i]=val[k];
    }
    
    for (i=0;i<n;i++) {
         val[i]=ival[i];
        ival[i]=0;
    }
    
    for (s = 1; s <= log2N; ++s) {
        m = 1 << s; 
        m2 = m >> 1;
        w.re=1;
        w.im=0;
        
        wm.re=rcos(PI/m2);
        wm.im=-rsin(PI/m2);

        for (j = 0; j < m2; ++j) {
            for (k = j; k < n; k += m) {
                t.re=(w.re*val[k+m2] - w.im*ival[k+m2]);
                t.im=(w.im*val[k+m2] + w.re*ival[k+m2]);
                u.re=val[k];
                u.im=ival[k];
                 val[k] = (int16_t)(u.re + t.re);
                ival[k] = (int16_t)(u.im + t.im);
                 val[k+m2] = (int16_t)(u.re - t.re);
                ival[k+m2] = (int16_t)(u.im - t.im);
            }
            t.re=(w.re*wm.re-w.im*wm.im);
            t.im=(w.im*wm.re+w.re*wm.im);
            w.re=t.re;
            w.im=t.im;
        }
    }
    
    int16_t valmax=-1;
    val[0]=0; // remove dc offset
    for (i=0;i<n;i++) {
        if(val[i]<0) val[i]=-val[i];
        if(valmax<val[i]) valmax=val[i];
    }
    return valmax;
}
