/*
** STM32F103RCT6 链接脚本 (256KB Flash, 48KB RAM)
** 与 Keil 项目 Flash 分区一致：最后 2KB 留给用户数据 (页62~63)
*/

/* 最高地址 0x0803FFFF (256KB Flash) */
/* 用户数据区占用 0x0803F800 ~ 0x0803FFFF (2KB, 页62~63) */
/* 应用程序可使用 0x08000000 ~ 0x0803F7FF */

MEMORY
{
  FLASH (rx)      : ORIGIN = 0x08000000, LENGTH = 254K
  RAM (xrw)       : ORIGIN = 0x20000000, LENGTH = 48K
}

/* 栈顶地址 = RAM 起始 + RAM 大小 */
_estack = ORIGIN(RAM) + LENGTH(RAM);

/* 最小栈/堆大小 */
_Min_Heap_Size = 0x200;      /* 512 字节堆 */
_Min_Stack_Size = 0x400;     /* 1KB 栈 */

SECTIONS
{
  /* 中断向量表必须放在 Flash 起始 */
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector))
    . = ALIGN(4);
  } >FLASH

  /* 代码段 */
  .text :
  {
    . = ALIGN(4);
    *(.text)
    *(.text*)
    *(.glue_7)
    *(.glue_7t)
    *(.eh_frame)

    KEEP (*(.init))
    KEEP (*(.fini))

    . = ALIGN(4);
    _etext = .;
  } >FLASH

  /* 常量数据 */
  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)
    *(.rodata*)
    . = ALIGN(4);
  } >FLASH

  .ARM.extab   : { *(.ARM.extab* .gnu.linkonce.armextab.*) } >FLASH
  .ARM : {
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
  } >FLASH

  .preinit_array     :
  {
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
  } >FLASH
  .init_array :
  {
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
  } >FLASH
  .fini_array :
  {
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array*))
    PROVIDE_HIDDEN (__fini_array_end = .);
  } >FLASH

  /* 供 __libc_init_array 使用 */
  _sidata = LOADADDR(.data);

  /* 已初始化数据段 */
  .data :
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data)
    *(.data*)

    . = ALIGN(4);
    _edata = .;
  } >RAM AT> FLASH

  /* 未初始化数据段 (BSS) */
  .bss :
  {
    . = ALIGN(4);
    _sbss = .;
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)

    . = ALIGN(4);
    _ebss = .;
    __bss_end__ = _ebss;
  } >RAM

  /* 用户堆栈段，由链接器检查可用空间 */
  ._user_heap_stack :
  {
    . = ALIGN(8);
    PROVIDE ( end = . );
    PROVIDE ( _end = . );
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } >RAM
}