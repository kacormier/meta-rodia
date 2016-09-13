/*
 *
 *  Copyright (c) 2015 RCATSOne Incorporated.  All rights reserved.
 *
 */
#ifdef __cplusplus
extern "C" {
#endif

// AMP instances have ph=3,4,5
#define AMP_PHONE_MIN 3
#define AMP_PHONE_MAX 5

int openFPGARegisterAccess(void);
int peekFPGA(unsigned int ph, unsigned int addr, unsigned char *value);
int pokeFPGA(unsigned int ph, unsigned int addr, unsigned char value);
int updateBitsFPGA(unsigned int ph, unsigned int addr, unsigned char bits, int setUnset);

#ifdef __cplusplus
}
#endif
