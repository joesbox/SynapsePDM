/*
  Linker script for full-flash application (no bootloader offset).
  STM32F446ZE: 512K flash total, 128K RAM total.
*/

ENTRY(Reset_Handler)

MEMORY
{
  RAM (xrw)  : ORIGIN = 0x20000000, LENGTH = 128K
  FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 512K
}

_estack = ORIGIN(RAM) + LENGTH(RAM);
_Min_Heap_Size = 0x200;
_Min_Stack_Size = 0x400;

SECTIONS
{
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector))
    . = ALIGN(4);
  } > FLASH

  .text :
  {
    . = ALIGN(4);
    *(.text)
    *(.text*)
    *(.glue_7)
    *(.glue_7t)
    *(.eh_frame)

    KEEP(*(.init))
    KEEP(*(.fini))

    . = ALIGN(4);
    _etext = .;
  } > FLASH

  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)
    *(.rodata*)
    . = ALIGN(4);
  } > FLASH

  .ARM.extab (READONLY) :
  {
    . = ALIGN(4);
    *(.ARM.extab* .gnu.linkonce.armextab.*)
    . = ALIGN(4);
  } > FLASH

  .ARM (READONLY) :
  {
    . = ALIGN(4);
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
    . = ALIGN(4);
  } > FLASH

  .preinit_array (READONLY) :
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN(__preinit_array_start = .);
    KEEP(*(.preinit_array*))
    PROVIDE_HIDDEN(__preinit_array_end = .);
    . = ALIGN(4);
  } > FLASH

  .init_array (READONLY) :
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN(__init_array_start = .);
    KEEP(*(SORT(.init_array.*)))
    KEEP(*(.init_array*))
    PROVIDE_HIDDEN(__init_array_end = .);
    . = ALIGN(4);
  } > FLASH

  .fini_array (READONLY) :
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN(__fini_array_start = .);
    KEEP(*(SORT(.fini_array.*)))
    KEEP(*(.fini_array*))
    PROVIDE_HIDDEN(__fini_array_end = .);
    . = ALIGN(4);
  } > FLASH

  _sidata = LOADADDR(.data);

  .data :
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data)
    *(.data*)
    *(.RamFunc)
    *(.RamFunc*)
    . = ALIGN(4);
  } > RAM AT > FLASH

  .tdata : ALIGN(4)
  {
    *(.tdata .tdata.* .gnu.linkonce.td.*)
    . = ALIGN(4);
    _edata = .;
    PROVIDE(__data_end = .);
    PROVIDE(__tdata_end = .);
  } > RAM AT > FLASH

  PROVIDE(__tdata_start = ADDR(.tdata));
  PROVIDE(__tdata_size = __tdata_end - __tdata_start);

  PROVIDE(__data_start = ADDR(.data));
  PROVIDE(__data_size = __data_end - __data_start);

  PROVIDE(__tdata_source = LOADADDR(.tdata));
  PROVIDE(__tdata_source_end = LOADADDR(.tdata) + SIZEOF(.tdata));
  PROVIDE(__tdata_source_size = __tdata_source_end - __tdata_source);

  PROVIDE(__data_source = LOADADDR(.data));
  PROVIDE(__data_source_end = __tdata_source_end);
  PROVIDE(__data_source_size = __data_source_end - __data_source);

  .tbss (NOLOAD) : ALIGN(4)
  {
    _sbss = .;
    __bss_start__ = _sbss;
    *(.tbss .tbss.*)
    . = ALIGN(4);
    PROVIDE(__tbss_end = .);
  } > RAM

  PROVIDE(__tbss_start = ADDR(.tbss));
  PROVIDE(__tbss_size = __tbss_end - __tbss_start);
  PROVIDE(__tbss_offset = ADDR(.tbss) - ADDR(.tdata));

  PROVIDE(__tls_base = __tdata_start);
  PROVIDE(__tls_end = __tbss_end);
  PROVIDE(__tls_size = __tls_end - __tls_base);
  PROVIDE(__tls_align = MAX(ALIGNOF(.tdata), ALIGNOF(.tbss)));
  PROVIDE(__tls_size_align = (__tls_size + __tls_align - 1) & ~(__tls_align - 1));
  PROVIDE(__arm32_tls_tcb_offset = MAX(8, __tls_align));
  PROVIDE(__arm64_tls_tcb_offset = MAX(16, __tls_align));

  .bss (NOLOAD) : ALIGN(4)
  {
    *(.bss)
    *(.bss*)
    *(COMMON)

    . = ALIGN(4);
    _ebss = .;
    __bss_end__ = _ebss;
  } > RAM

  ._user_heap_stack (NOLOAD) :
  {
    . = ALIGN(8);
    PROVIDE(end = .);
    PROVIDE(_end = .);
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } > RAM

  /DISCARD/ :
  {
    libc.a(*);
    libm.a(*);
    libgcc.a(*);
  }

  .ARM.attributes 0 : { *(.ARM.attributes) }
}
