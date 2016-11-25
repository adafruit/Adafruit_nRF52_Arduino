#ifndef DLL_COMMON_DEFINITIONS_H
#define DLL_COMMON_DEFINITIONS_H



#if defined(__cplusplus)
extern "C" {
#endif


#define MIN_JLINK_MAJOR_VERSION                         (5UL)
#define MIN_JLINK_MINOR_VERSION                         (2UL)

#define JLINKARM_SWD_MIN_SPEED_KHZ                      (125UL)
#define JLINKARM_SWD_DEFAULT_SPEED_KHZ                  (2000UL)
#define JLINKARM_SWD_MAX_SPEED_KHZ                      (50000UL)


/* RAM power status declarations. */
#define MAX_RAM_BLOCKS      (16)                    /* NRF52 FP1 has 16 ram blocks. */
    
typedef enum {
    RAM_OFF = 0,
    RAM_ON,
} ram_section_power_status_t;


/* Identified device versions of nRF devices. */
typedef enum {
    R0 = 0,
    R1,
    R2,
    R3,
    R4,
    R5,
    R6,
    R7,
    R8,
    R9,
    R10,
    R11,
    R12,
    R13,
    R14,
    R15,
    XPSR,
    MSP,
    PSP 
} cpu_registers_t;


/* Possible readback protection status. */
typedef enum {
    NONE = 0,
    REGION_0,
    ALL,
    BOTH,
} readback_protection_status_t;


/* Possible region 0 source. */
typedef enum {
    NO_REGION_0 = 0,
    FACTORY,
    USER,
} region_0_source_t;


/* Identified device versions of nRF devices. */
typedef enum {
    UNKNOWN,
    NRF51_XLR1,
    NRF51_XLR2,
    NRF51_XLR3,
    NRF51_L3,
    NRF51_XLR3P,
    NRF51_XLR3LC,
    NRF52_FP1_ENGA,
    NRF52_FP1_ENGB,
    NRF52_FP1
} device_version_t;

/* Identified types of nRF devices */
typedef enum {
    NRF51_FAMILY,
    NRF52_FAMILY
} device_family_t;

/* Possible rtt channel directions */
typedef enum {
    UP_DIRECTION = 0,
    DOWN_DIRECTION = 1
} rtt_direction_t;

/* Every DLL function has either a void or nrfjprogdll_err_t return type. */
typedef enum
{
    SUCCESS                                     = 0,
    
    /* PC Issues */
    OUT_OF_MEMORY                               = -1, 
    
    /* Wrong use of dll errors. */
    INVALID_OPERATION                           = -2,
    INVALID_PARAMETER                           = -3,
    INVALID_DEVICE_FOR_OPERATION                = -4,
    WRONG_FAMILY_FOR_DEVICE                     = -5,

    
    /* Connexion issues. */
    EMULATOR_NOT_CONNECTED                      = -10,
    CANNOT_CONNECT                              = -11,
    LOW_VOLTAGE                                 = -12,
    NO_EMULATOR_CONNECTED                       = -13,

    /* Device issues. */
    NVMC_ERROR                                  = -20,

    /* Operation not available. */
    NOT_AVAILABLE_BECAUSE_PROTECTION            = -90,
    NOT_AVAILABLE_BECAUSE_MPU_CONFIG            = -91,
    
    /* JlinkARM DLL errors. */
    JLINKARM_DLL_NOT_FOUND                      = -100,
    JLINKARM_DLL_COULD_NOT_BE_OPENED            = -101,
    JLINKARM_DLL_ERROR                          = -102,
    JLINKARM_DLL_TOO_OLD                        = -103,

    /* nrfjprog sub DLL errors. */
    NRFJPROG_SUB_DLL_NOT_FOUND                  = -150,
    NRFJPROG_SUB_DLL_COULD_NOT_BE_OPENED        = -151,
    
    /* Not implemented. */
    NOT_IMPLEMENTED_ERROR                       = -255,

} nrfjprogdll_err_t;


#if defined(__cplusplus)
}
#endif

#endif /* DLL_COMMON_DEFINITIONS_H */