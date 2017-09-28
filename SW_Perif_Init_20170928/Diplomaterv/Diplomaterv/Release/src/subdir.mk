################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/adc.c \
../src/gpio.c \
../src/main.c \
../src/stm32f3xx_hal_msp.c \
../src/stm32f3xx_it.c \
../src/tim.c 

OBJS += \
./src/adc.o \
./src/gpio.o \
./src/main.o \
./src/stm32f3xx_hal_msp.o \
./src/stm32f3xx_it.o \
./src/tim.o 

C_DEPS += \
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
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -flto -Wunused -Wuninitialized -Wall -Wextra -Wmissing-declarations -Wconversion -Wpointer-arith -Wpadded -Wshadow -Wlogical-op -Waggregate-return -Wfloat-equal  -g -DOS_USE_TRACE_SEMIHOSTING_DEBUG -DSTM32F30X -DUSE_STDPERIPH_DRIVER -DHSE_VALUE=32000000 -DUSE_HAL_DRIVER -DSTM32F303xC -I"../include" -I"../system/include" -I"../system/include/cmsis" -I"../system/include/stm32f3-stdperiph" -I"D:\Diplomaterv\Diplomaterv\include" -I"D:\Diplomaterv\Diplomaterv\Drivers\CMSIS\Include" -I"D:\Diplomaterv\Diplomaterv\Drivers\CMSIS\Device\ST\STM32F3xx\Include" -I"D:\Diplomaterv\Diplomaterv\Drivers\STM32F3xx_HAL_Driver\Inc" -std=gnu11 -Wmissing-prototypes -Wstrict-prototypes -Wbad-function-cast -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


