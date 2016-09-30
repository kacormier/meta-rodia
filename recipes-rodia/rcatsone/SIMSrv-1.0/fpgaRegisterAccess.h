/*
 *
 *  Copyright (c) 2015 RCATSOne Incorporated.  All rights reserved.
 *
 */
#ifdef __cplusplus
extern "C" {
#endif

int peekFPGA(unsigned int ph, unsigned int addr, unsigned char *value);
int pokeFPGA(unsigned int ph, unsigned int addr, unsigned char value);
int updateBitsFPGA(unsigned int ph, unsigned int addr, unsigned char bits, int setUnset);

#ifdef __cplusplus
}
#endif
