source gdb-dashboard/.gdbinit
source svd-tools/gdb-svd.py
#source gdb-dashboard/.gdbinit
dashboard svd load svd/STM32F427.svd
svd svd/STM32F427.svd
# Connect to OpenOCD
target extended-remote :3333

# Enable OpenOCD's semihosting capability
monitor arm semihosting enable
monitor arm semihosting_fileio enable

# Set backtrace limit to not have infinite backtrace loops
set backtrace limit 32

# Print demangled symbols
set print asm-demangle on

# Print 5 instructions every time we break.
# Note that `layout asm` is also pretty good, but my up arrow doesn't work
# anymore in this mode, so I prefer display/5i.
#display/5i $pc

# Write our program into the device's internal flash
monitor reset halt
#symbol-file firmware/f427.elf
file /home/goran/Projects/reverse_mono_6ks/build/reverse_mono_6ks.elf
load /home/goran/Projects/reverse_mono_6ks/build/reverse_mono_6ks.elf
break main
break LCD_panel_half_black
break readInput
#break xpt2046_getXY
#break xpt2046_read
#break SPI_EMUL_TX_DMA_IRQHandler
#break SPI_EMUL_RX_DMA_IRQHandler
#break SPI_Emul_Transmission_Process_Complete
#break HAL_SPI_Emul_TransmitReceive_DMA
#break TIM4_IRQHandler
#break HAL_TIM_Base_MspInit
#break HAL_NVIC_DisableIRQ
#break HAL_NVIC_EnableIRQ
#break SPI_Emul_SetConfig_DMATx
#break SPI_Emul_SetConfig_DMARx
#break xpt2046_getXY
#break EXTI9_5_IRQHandler
#break lv_obj_refr_size
#break tft_flush_cb
#break DMA_TransferComplete
#break *0x08000ac4
#break SysTick_Handler
tui enable
#layout asm
tui reg general
info registers
#disassemble 0x08000ac4, +20 
# Resume execution
#continue
