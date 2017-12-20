/* Linker script to configure memory regions. */

SEARCH_DIR(.)
GROUP(-lgcc -lc -lnosys)

MEMORY
{
  FLASH (rx) : ORIGIN = 0x23000, LENGTH = 0x25000 /* Max 148 KB for dual banks */

  /* SRAM required by S132 depend on
   * - Attribute Table Size 0x800
   * - Vendor UUID count    10
   * - Max ATT MTU is 247
   * - Role concurrent connection peripherl (1) + central (0) + secure links (1)
   * - Event Len = 3, HVN queue = 3, Write CMD queue = 0
   */ 
  /* RAM (rwx) :  ORIGIN = 0x20003000, LENGTH = 0x20010000 - 0x20003000  */
  RAM (rwx) :  ORIGIN = 0x20003800, LENGTH = 0x20010000 - 0x20003800
  
  /* Reference configuration
     - MTU = 247, hvn = 7, wrcmd = 3 , evt len = 3
          ram_start = 0x20003310

     - MTU = 247, hvn = 3, wrcmd = 3, evt len = 3
          ram_start = 0x20002F10

     - MTU = 23, hvn = 7, wrcmd = 3, evt len = 3
        ram_start = 0x200025A0
  */
}

SECTIONS
{
  . = ALIGN(4);
  .svc_data :
  {
    PROVIDE(__start_svc_data = .);
    KEEP(*(.svc_data))
    PROVIDE(__stop_svc_data = .);
  } > RAM
  
  .fs_data :
  {
    PROVIDE(__start_fs_data = .);
    KEEP(*(.fs_data))
    PROVIDE(__stop_fs_data = .);
  } > RAM
} INSERT AFTER .data;

INCLUDE "nrf5x_common.ld"
