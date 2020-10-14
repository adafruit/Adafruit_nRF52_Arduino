#ifndef NRFX_CONFIG_H__
#define NRFX_CONFIG_H__

#define NRFX_POWER_ENABLED              1
#define NRFX_POWER_DEFAULT_CONFIG_IRQ_PRIORITY  7

#define NRFX_CLOCK_ENABLED 0

#define NRFX_SPIM_ENABLED            1
#define NRFX_SPIM_MISO_PULL_CFG      1 // pulldown
#define NRFX_SPIM_EXTENDED_ENABLED   0

#define NRFX_SPIM0_ENABLED           0 // used as I2C
#define NRFX_SPIM1_ENABLED           0 // used as I2C
#define NRFX_SPIM2_ENABLED           1

#ifdef NRF52840_XXAA
  #define NRFX_QSPI_ENABLED   1
  #define NRFX_SPIM3_ENABLED  1
#elif defined(NRF52833_XXAA)
  #define NRFX_QSPI_ENABLED   0
  #define NRFX_SPIM3_ENABLED  1
#else
  #define NRFX_QSPI_ENABLED   0
  #define NRFX_SPIM3_ENABLED  0
#endif


#endif // NRFX_CONFIG_H__
