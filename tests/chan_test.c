#include <stdio.h>
#include <math.h>
#include "v22bs_channel.h"
int main(void){
    v22bs_channel_t ch; int16_t out; int i;
    double meansq=0,mean=0; long n=0; int clip=0;
    v22bs_channel_init(&ch, 99);
    ch.attenuation=1.0; ch.sigma=300;
    for(i=0;i<200000;i++){
        out=v22bs_channel_step(&ch,0);
        meansq+=(double)out*out; mean+=(double)out; n++;
        if(out<=-32768||out>=32767) clip++;
    }
    printf("noise-only sigma=300: rms=%.2f mean=%.3f clip_pct=%.3f\n",
           sqrt(meansq/n), mean/n, 100.0*clip/n);
    /* distribution check */
    ch.noise_state=42; meansq=0; n=0; clip=0;
    for(i=0;i<50000;i++){
        out=v22bs_channel_step(&ch,0);
        meansq+=(double)out*out; n++;
        if(out<=-32768||out>=32767) clip++;
    }
    printf("noise-only sigma=300 (seed42): rms=%.2f clip_pct=%.4f\n",
           sqrt(meansq/n), 100.0*clip/n);
    return 0;
}
