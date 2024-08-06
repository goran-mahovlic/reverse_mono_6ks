# Reversing Mono 6Ks

## MCU

GD32F427ZG

## LCD

ILI9341 with resolution 480x320 with FMC connection to GD32F427

LCD Register select is connected to PG7 so it needs to be driven manually

## TOUCH

Is not connected to SPI capable pins - so softSPI will be needed

## FLASH

Winbond W25Q128 is connected to SPI1


