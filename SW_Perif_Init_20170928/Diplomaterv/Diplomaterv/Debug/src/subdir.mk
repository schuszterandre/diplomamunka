################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/LCD.c \
../src/adc.c \
../src/gpio.c \
../src/main.c \
../src/stm32f3xx_hal_msp.c \
../src/stm32f3xx_it.c \
../src/tim.c 

OBJS += \
./src/LCD.o \
./src/adc.o \
./src/gpio.o \
./src/main.o \
./src/stm32f3xx_hal_msp.o \
./src/stm32f3xx_it.o \
./src/tim.o 

C_DEPS += \
./src/LCD.d \
./src/adc.d \
./src/gpio.d \
./src/main.d \
./src/stm32f3xx_hal_msp.d \
./src/stm32f3xx_it.d \
./src/tim.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -flto -fno-move-loop-invariants -Wunused -Wuninitialized -Wall -Wextra -Wmissing-declarations -Wconversion -Wpointer-arith -Wpadded -Wshadow -Wlogical-op -Waggregate-return -Wfloat-equal  -g3 -DUSE_HAL_DRIVER -DSTM32F303xC -DHSE_VALUE=32000000 -I"D:\Diplomaterv\Diplomaterv\include" -I"D:\Diplomaterv\Diplomaterv\Drivers\CMSIS\Include" -I"D:\Diplomaterv\Diplomaterv\Drivers\CMSIS\Device\ST\STM32F3xx\Include" -I"D:\Diplomaterv\Diplomaterv\Drivers\STM32F3xx_HAL_Driver\Inc" -std=gnu11 -Wmissing-prototypes -Wstrict-prototypes -Wbad-function-cast -ffunction-sections -fdata-sections -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


