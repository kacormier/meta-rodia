/*
 *
 *  Copyright (c) 2001 - 2006 Casabyte, Inc.  All rights reserved.
 *
 */
#ifndef __CRES_H__

#define __CRES_H__

typedef enum {
    INVALID_RESOURCE,
    PHNARES,
    PHNBRES,
    PHNCRES,
    PHNDRES,
    PHNERES,
    SCRNRES,
    FLAGSRES,
    DTMFRES,
    PHASPRES,
    PHBSPRES,
    PHCSPRES,
    NVRAMRES,
    SMSRESA,
    SMSRESB,
    SMSRESC,
    PHARSTRE,
    PHBRSTRE,
    PHCRSTRE,
    FPGA_POKE,
    G20RESPORTA,        // G20 phone 1 device file handle (com port) mutex
    G20RESPORTB,        // G20 phone 2 device file handle (com port) mutex
    G20RESPORTC,        // G20 phone 3 device file handle (com port) mutex
    SEMA_ALLOCATOR_RES, // for dynamic semaphore allocator

    FIRST_DYNAMIC_RESOURCE,   // this must be the last resource entry (it is used for making dynamic resources)!
    MAX_RESOURCE = 128        // This is used for checking the upper bound of the resources.
} ResourceID;

#endif

