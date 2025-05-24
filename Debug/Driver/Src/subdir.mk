################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Driver/Src/stm32f411xx_gpio_driver.c 

OBJS += \
./Driver/Src/stm32f411xx_gpio_driver.o 

C_DEPS += \
./Driver/Src/stm32f411xx_gpio_driver.d 


# Each subdirectory must supply rules for building sources it contributes
Driver/Src/%.o Driver/Src/%.su Driver/Src/%.cyclo: ../Driver/Src/%.c Driver/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DNUCLEO_F411RE -DSTM32 -DSTM32F4 -DSTM32F411RETx -c -I../Inc -I"D:/Study_Materials/Embedded Systems/Pantech Solutions/Pantech Task/STM32 IDE/HC-05_Interface_STM32_InBuiltLedControl/Driver/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Driver-2f-Src

clean-Driver-2f-Src:
	-$(RM) ./Driver/Src/stm32f411xx_gpio_driver.cyclo ./Driver/Src/stm32f411xx_gpio_driver.d ./Driver/Src/stm32f411xx_gpio_driver.o ./Driver/Src/stm32f411xx_gpio_driver.su

.PHONY: clean-Driver-2f-Src

