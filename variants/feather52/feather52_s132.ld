/* Linker script to configure memory regions. */

SEARCH_DIR(.)
GROUP(-lgcc -lc -lnosys)

MEMORY
{
  FLASH (rx) : ORIGIN = 0x1c000, LENGTH = 0x64000
  
  /* ATTR SIZE = 0x0580, (central, prph, sec)  = (0, 1, 0) */
  /* RAM (rwx) :  ORIGIN = 0x20002080, LENGTH = 0xdf80 */
  
  /* ATTR SIZE = 0x1000, (central, prph, sec) = (0, 1, 0) */
  /* RAM (rwx) :  ORIGIN = 0x20002B00, LENGTH = 0xD500 */
  
  /* ATTR SIZE = 0x0B00, (central, prph, sec) = (0, 1, 0) */
  /*RAM (rwx) :  ORIGIN = 0x20001B00 + 0x0B00, LENGTH = 0xE500 - 0x0B00*/
    
  /* ATTR SIZE = 0x0B00, (central, prph, sec) = (4, 1, 1) */
  RAM (rwx) :  ORIGIN = 0x20001B00 + 0x0B00 + 0x1280, LENGTH = 0xE500 - 0x0B00 - 0x1280
}

SECTIONS
{
  .fs_data :
  {
    PROVIDE(__start_fs_data = .);
    KEEP(*(.fs_data))
    PROVIDE(__stop_fs_data = .);
  } > RAM
} INSERT AFTER .data;

INCLUDE "nrf5x_common.ld"

/* S132 v2.0.1 role configuration SRAM usage */
/* APP_RAM_BASE_CENTRAL_LINKS_0_PERIPH_LINKS_1_SEC_COUNT_0_MID_BW 0x20001fe8 */

/* APP_RAM_BASE_CENTRAL_LINKS_4_PERIPH_LINKS_1_SEC_COUNT_1_MID_BW 0x20003268 inc = 1280 */
/* APP_RAM_BASE_CENTRAL_LINKS_4_PERIPH_LINKS_1_SEC_COUNT_2_MID_BW 0x20003438 inc = 1450 */
/* APP_RAM_BASE_CENTRAL_LINKS_4_PERIPH_LINKS_1_SEC_COUNT_4_MID_BW 0x200037d0 inc = 17e8 */


/* APP_RAM_BASE_CENTRAL_LINKS_5_PERIPH_LINKS_1_SEC_COUNT_5_MID_BW 0x20003dc8 inc = 1de0*/
/* APP_RAM_BASE_CENTRAL_LINKS_6_PERIPH_LINKS_1_SEC_COUNT_6_MID_BW 0x200043c0 inc = 23d8*/
/* APP_RAM_BASE_CENTRAL_LINKS_7_PERIPH_LINKS_1_SEC_COUNT_7_MID_BW 0x200049b8 inc = 29d0*/
