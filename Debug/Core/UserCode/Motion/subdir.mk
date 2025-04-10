################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/UserCode/Motion/motion.c 

OBJS += \
./Core/UserCode/Motion/motion.o 

C_DEPS += \
./Core/UserCode/Motion/motion.d 


# Each subdirectory must supply rules for building sources it contributes
Core/UserCode/Motion/%.o Core/UserCode/Motion/%.su Core/UserCode/Motion/%.cyclo: ../Core/UserCode/Motion/%.c Core/UserCode/Motion/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-UserCode-2f-Motion

clean-Core-2f-UserCode-2f-Motion:
	-$(RM) ./Core/UserCode/Motion/motion.cyclo ./Core/UserCode/Motion/motion.d ./Core/UserCode/Motion/motion.o ./Core/UserCode/Motion/motion.su

.PHONY: clean-Core-2f-UserCode-2f-Motion

