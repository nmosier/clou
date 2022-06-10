/************************************************

The Tiny Encryption Algorithm (TEA) by 
David Wheeler and Roger Needham of the
Cambridge Computer Laboratory

**** ANSI C VERSION ****

Notes:

TEA is a Feistel cipher with XOR and
and addition as the non-linear mixing
functions. 

Takes 64 bits of data in v[0] and v[1].
Returns 64 bits of data in w[0] and w[1].
Takes 128 bits of key in k[0] - k[3].

TEA can be operated in any of the modes
of DES. Cipher Block Chaining is, for example,
simple to implement.

n is the number of iterations. 32 is ample,
16 is sufficient, as few as eight may be OK.
The algorithm achieves good dispersion after
six iterations. The iteration count can be
made variable if required.

Note this is optimised for 32-bit CPUs with
fast shift capabilities. It can very easily
be ported to assembly language on most CPUs.

delta is chosen to be the real part of (the
golden ratio Sqrt(5/4) - 1/2 ~ 0.618034
multiplied by 2^32). 

Source: https://www.schneier.com/sccd/TEA.C

************************************************/

void encipher(unsigned long *const v,unsigned long *const w,
const unsigned long *const k)
{
   register unsigned long       y=v[0],z=v[1],sum=0,delta=0x9E3779B9,
				a=k[0],b=k[1],c=k[2],d=k[3],n=32;

   while(n-->0)
      {
      sum += delta;
      y += (z<<4)+a ^ z+sum ^ (z>>5)+b;
      z += (y<<4)+c ^ y+sum ^ (y>>5)+d;
      }

   w[0]=y; w[1]=z;
}

void decipher(unsigned long *const v,unsigned long *const w,
const unsigned long *const k)
{
   register unsigned long       y=v[0],z=v[1],sum=0xC6EF3720,
				delta=0x9E3779B9,a=k[0],b=k[1],c=k[2],
				d=k[3],n=32;

   /* sum = delta<<5, in general sum = delta * n */

   while(n-->0)
      {
      z -= (y<<4)+c ^ y+sum ^ (y>>5)+d;
      y -= (z<<4)+a ^ z+sum ^ (z>>5)+b;
      sum -= delta;
      }
   
   w[0]=y; w[1]=z;
}
