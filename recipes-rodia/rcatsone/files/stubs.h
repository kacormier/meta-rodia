#define DBG2_SIM_SIMU 0
#define ResultCode int
#define KS_delay(...)
#define TRUE 1
#define FALSE 0
#define ERROR_OK 0
#define debug(...)
#define ddebug2(...)
#define MAKE_RCATS_ERROR(...) 1
#define NUM_SIM_SLOTS 4
#define eFATAL       1
#define eWARN        2
#define error(...)
typedef int RESOURCE;

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