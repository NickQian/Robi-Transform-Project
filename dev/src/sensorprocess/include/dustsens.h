/*  c driver for dust sensor module( ) of RTP project
    ----
    Licensed under BSD license.
    by Nick Qian
    ----
    0.1 - 2017.3.9  init version for Raspberry 2B(bcm2836)
    ----
    output: dust info inquiried by sens.py
    input: sensor module(DSM501A) 
*/

#ifndef _DUSTSENS_H_
  #define _DUSTSENS_H_

#define USE_DSM501A


#ifdef USE_DSM501A
  #pragma message("@Compile MSG@: using DSM501A as dust sensor")

//....


#endif   // ifdef USE_DSM501A



#endif
