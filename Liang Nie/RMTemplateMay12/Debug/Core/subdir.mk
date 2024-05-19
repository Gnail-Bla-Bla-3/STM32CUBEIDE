################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/bsp_imu_pwm.c \
../Core/imu_temp_control_task.c \
../Core/pid.c 

OBJS += \
./Core/bsp_imu_pwm.o \
./Core/imu_temp_control_task.o \
./Core/pid.o 

C_DEPS += \
./Core/bsp_imu_pwm.d \
./Core/imu_temp_control_task.d \
./Core/pid.d 


# Each subdirectory must supply rules for building sources it contributes
Core/%.o Core/%.su Core/%.cyclo: ../Core/%.c Core/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core

clean-Core:
	-$(RM) ./Core/bsp_imu_pwm.cyclo ./Core/bsp_imu_pwm.d ./Core/bsp_imu_pwm.o ./Core/bsp_imu_pwm.su ./Core/imu_temp_control_task.cyclo ./Core/imu_temp_control_task.d ./Core/imu_temp_control_task.o ./Core/imu_temp_control_task.su ./Core/pid.cyclo ./Core/pid.d ./Core/pid.o ./Core/pid.su

.PHONY: clean-Core

