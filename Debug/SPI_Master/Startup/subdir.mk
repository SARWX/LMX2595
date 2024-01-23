################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../SPI_Master/Startup/startup_stm32f103c8tx.s 

OBJS += \
./SPI_Master/Startup/startup_stm32f103c8tx.o 

S_DEPS += \
./SPI_Master/Startup/startup_stm32f103c8tx.d 


# Each subdirectory must supply rules for building sources it contributes
SPI_Master/Startup/%.o: ../SPI_Master/Startup/%.s SPI_Master/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m3 -g3 -DDEBUG -c -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@" "$<"

