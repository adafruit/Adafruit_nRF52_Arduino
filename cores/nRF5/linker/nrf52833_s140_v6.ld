/* Linker script to configure memory regions. */

SEARCH_DIR(.)
GROUP(-lgcc -lc -lnosys)

MEMORY
{
  /* 0x6d000 end address comes from bootloader flash starting at 0x74000
  and LFS_FLASH_ADDR with LFS_FLASH_TOTAL_SIZE * 4096 bytes sitting below that address.
  */
  FLASH (rx)     : ORIGIN = 0x26000, LENGTH = 0x6D000 - 0x26000

  /* SRAM required by S140 depend on
   * - Attribute Table Size (Number of Services and Characteristics)
   * - Vendor UUID count
   * - Max ATT MTU
   * - Concurrent connection peripheral + central + secure links
   * - Event Len, HVN queue, Write CMD queue
   */ 
  RAM (rwx) :  ORIGIN = 0x20006000, LENGTH = 0x20020000 - 0x20006000
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

INCLUDE "nrf52_common.ld"
