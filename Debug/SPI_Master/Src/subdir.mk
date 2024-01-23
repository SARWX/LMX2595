################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../SPI_Master/Src/main.c \
../SPI_Master/Src/stm32f1xx_hal_msp.c \
../SPI_Master/Src/stm32f1xx_it.c \
../SPI_Master/Src/syscalls.c \
../SPI_Master/Src/sysmem.c \
../SPI_Master/Src/system_stm32f1xx.c 

OBJS += \
./SPI_Master/Src/main.o \
./SPI_Master/Src/stm32f1xx_hal_msp.o \
./SPI_Master/Src/stm32f1xx_it.o \
./SPI_Master/Src/syscalls.o \
./SPI_Master/Src/sysmem.o \
./SPI_Master/Src/system_stm32f1xx.o 

C_DEPS += \
./SPI_Master/Src/main.d \
./SPI_Master/Src/stm32f1xx_hal_msp.d \
./SPI_Master/Src/stm32f1xx_it.d \
./SPI_Master/Src/syscalls.d \
./SPI_Master/Src/sysmem.d \
./SPI_Master/Src/system_stm32f1xx.d 


# Each subdirectory must supply rules for building sources it contributes
SPI_Master/Src/%.o: ../SPI_Master/Src/%.c SPI_Master/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

