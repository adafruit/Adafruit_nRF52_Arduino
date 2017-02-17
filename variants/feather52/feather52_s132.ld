/* Linker script to configure memory regions. */

SEARCH_DIR(.)
GROUP(-lgcc -lc -lnosys)

MEMORY
{
  FLASH (rx) : ORIGIN = 0x1c000, LENGTH = 0x64000
  
  /* ATTR SIZE = 0x0580, mid bandwidth (central, prph, sec)  = (0, 1, 0) */
  /* RAM (rwx) :  ORIGIN = 0x20002080, LENGTH = 0xdf80 */
  
  /* ATTR SIZE = 0x1000, mid bandwidth  (central, prph, sec) = (0, 1, 0) */
  /* RAM (rwx) :  ORIGIN = 0x20002B00, LENGTH = 0xD500 */
  
  /* ATTR SIZE = 0x0B00, mid bandwidth  (central, prph, sec) = (0, 1, 0) */
  RAM (rwx) :  ORIGIN = 0x20002600, LENGTH = 0xDA00
  
  /* ATTR SIZE = 0x0B00, mid bandwidth  (central, prph, sec) = (4, 1, 4) */
  /*RAM (rwx) :  ORIGIN = 0x20002600, LENGTH = 0xDA00*/
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