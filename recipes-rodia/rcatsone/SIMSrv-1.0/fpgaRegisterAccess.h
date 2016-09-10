/*
 *
 *  Copyright (c) 2015 RCATSOne Incorporated.  All rights reserved.
 *
 */
#ifdef __cplusplus
extern "C" {
#endif

int openFPGARegisterAccess(void);
int peekFPGA(unsigned int addr, unsigned char *value);
int pokeFPGA(unsigned int addr, unsigned char value);
int updateBitsFPGA(unsigned int addr, unsigned char bits, int setUnset);

#ifdef __cplusplus
}
#endif
