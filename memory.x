/* Memory layout for nRF52840 with Adafruit bootloader */
/* Bootloader + SoftDevice S140 occupy 0x00000000-0x00026000 */
MEMORY
{
  /* FLASH starts after bootloader + SoftDevice (152KB reserved) */
  FLASH : ORIGIN = 0x00026000, LENGTH = 1024K - 152K

  /* RAM offset required for Embassy framework */
  RAM : ORIGIN = 0x20002000, LENGTH = 248K
}

_stack_start = ORIGIN(RAM) + LENGTH(RAM);
