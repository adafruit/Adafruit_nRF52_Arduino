/* Linker script to configure memory regions. */

SEARCH_DIR(.)
GROUP(-lgcc -lc -lnosys)

MEMORY
{
  FLASH (rx) : ORIGIN = 0x23000, LENGTH = 0x25000 /* Max 148 KB for dual banks */
  RAM (rwx) :  ORIGIN = 0x20002000 + 0x800, LENGTH = 0xE000 - 0x800

  /* SRAM required by S132 depend on
   * - Attribute Table Size 0x800
   * - Vendor UUID count    20
   * - Role concurrent connection central + peripherl + secure links
   * - 
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
