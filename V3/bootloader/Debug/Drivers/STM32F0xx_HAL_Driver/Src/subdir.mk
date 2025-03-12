################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_ll_crc.c \
../Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_ll_dma.c \
../Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_ll_exti.c \
../Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_ll_gpio.c \
../Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_ll_i2c.c \
../Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_ll_pwr.c \
../Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_ll_rcc.c \
../Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_ll_utils.c 

OBJS += \
./Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_ll_crc.o \
./Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_ll_dma.o \
./Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_ll_exti.o \
./Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_ll_gpio.o \
./Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_ll_i2c.o \
./Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_ll_pwr.o \
./Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_ll_rcc.o \
./Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_ll_utils.o 

C_DEPS += \
./Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_ll_crc.d \
./Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_ll_dma.d \
./Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_ll_exti.d \
./Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_ll_gpio.d \
./Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_ll_i2c.d \
./Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_ll_pwr.d \
./Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_ll_rcc.d \
./Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_ll_utils.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/STM32F0xx_HAL_Driver/Src/%.o Drivers/STM32F0xx_HAL_Driver/Src/%.su: ../Drivers/STM32F0xx_HAL_Driver/Src/%.c Drivers/STM32F0xx_HAL_Driver/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DDEBUG -DUSE_FULL_LL_DRIVER -DHSE_VALUE=8000000 -DHSE_STARTUP_TIMEOUT=100 -DLSE_STARTUP_TIMEOUT=5000 -DLSE_VALUE=32768 -DHSI_VALUE=8000000 -DLSI_VALUE=40000 -DVDD_VALUE=3300 -DPREFETCH_ENABLE=1 -DINSTRUCTION_CACHE_ENABLE=0 -DDATA_CACHE_ENABLE=0 -DSTM32F030x6 -c -I../Core/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F0xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-STM32F0xx_HAL_Driver-2f-Src

clean-Drivers-2f-STM32F0xx_HAL_Driver-2f-Src:
	-$(RM) ./Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_ll_crc.d ./Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_ll_crc.o ./Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_ll_crc.su ./Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_ll_dma.d ./Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_ll_dma.o ./Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_ll_dma.su ./Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_ll_exti.d ./Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_ll_exti.o ./Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_ll_exti.su ./Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_ll_gpio.d ./Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_ll_gpio.o ./Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_ll_gpio.su ./Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_ll_i2c.d ./Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_ll_i2c.o ./Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_ll_i2c.su ./Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_ll_pwr.d ./Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_ll_pwr.o ./Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_ll_pwr.su ./Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_ll_rcc.d ./Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_ll_rcc.o ./Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_ll_rcc.su ./Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_ll_utils.d ./Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_ll_utils.o ./Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_ll_utils.su

.PHONY: clean-Drivers-2f-STM32F0xx_HAL_Driver-2f-Src

