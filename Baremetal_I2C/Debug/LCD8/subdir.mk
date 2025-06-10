################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../LCD8/lcd8.c 

OBJS += \
./LCD8/lcd8.o 

C_DEPS += \
./LCD8/lcd8.d 


# Each subdirectory must supply rules for building sources it contributes
LCD8/%.o LCD8/%.su LCD8/%.cyclo: ../LCD8/%.c LCD8/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F405xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/davidkumar/STM32CubeIDE/workspace_1.18.1/I2C_Start/LCD8" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-LCD8

clean-LCD8:
	-$(RM) ./LCD8/lcd8.cyclo ./LCD8/lcd8.d ./LCD8/lcd8.o ./LCD8/lcd8.su

.PHONY: clean-LCD8

