/* Linker script for ST STM32F303 nucleo (STM32F303K8T6, 64K flash, 16K RAM (Of which 4K are CCM RAM and 12K are SRAM). */
MEMORY
{
	rom (rx) : ORIGIN = 0x08000000, LENGTH = 64K
	ram (rwx) : ORIGIN = 0x20000000, LENGTH = 12K
}

INCLUDE libopencm3_stm32f3.ld
