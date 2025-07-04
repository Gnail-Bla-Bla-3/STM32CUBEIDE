################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/BMI088Middleware.c \
../Core/Src/BMI088driver.c \
../Core/Src/CAL_RM25.c \
../Core/Src/CAN.c \
../Core/Src/DBUS.c \
../Core/Src/SWERVE.c \
../Core/Src/SwerveBuffer.c \
../Core/Src/UART_RM25.c \
../Core/Src/bsp_imu_pwm.c \
../Core/Src/freertos.c \
../Core/Src/imu_temp_control_task.c \
../Core/Src/main.c \
../Core/Src/pid.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c 

OBJS += \
./Core/Src/BMI088Middleware.o \
./Core/Src/BMI088driver.o \
./Core/Src/CAL_RM25.o \
./Core/Src/CAN.o \
./Core/Src/DBUS.o \
./Core/Src/SWERVE.o \
./Core/Src/SwerveBuffer.o \
./Core/Src/UART_RM25.o \
./Core/Src/bsp_imu_pwm.o \
./Core/Src/freertos.o \
./Core/Src/imu_temp_control_task.o \
./Core/Src/main.o \
./Core/Src/pid.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o 

C_DEPS += \
./Core/Src/BMI088Middleware.d \
./Core/Src/BMI088driver.d \
./Core/Src/CAL_RM25.d \
./Core/Src/CAN.d \
./Core/Src/DBUS.d \
./Core/Src/SWERVE.d \
./Core/Src/SwerveBuffer.d \
./Core/Src/UART_RM25.d \
./Core/Src/bsp_imu_pwm.d \
./Core/Src/freertos.d \
./Core/Src/imu_temp_control_task.d \
./Core/Src/main.d \
./Core/Src/pid.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/BMI088Middleware.cyclo ./Core/Src/BMI088Middleware.d ./Core/Src/BMI088Middleware.o ./Core/Src/BMI088Middleware.su ./Core/Src/BMI088driver.cyclo ./Core/Src/BMI088driver.d ./Core/Src/BMI088driver.o ./Core/Src/BMI088driver.su ./Core/Src/CAL_RM25.cyclo ./Core/Src/CAL_RM25.d ./Core/Src/CAL_RM25.o ./Core/Src/CAL_RM25.su ./Core/Src/CAN.cyclo ./Core/Src/CAN.d ./Core/Src/CAN.o ./Core/Src/CAN.su ./Core/Src/DBUS.cyclo ./Core/Src/DBUS.d ./Core/Src/DBUS.o ./Core/Src/DBUS.su ./Core/Src/SWERVE.cyclo ./Core/Src/SWERVE.d ./Core/Src/SWERVE.o ./Core/Src/SWERVE.su ./Core/Src/SwerveBuffer.cyclo ./Core/Src/SwerveBuffer.d ./Core/Src/SwerveBuffer.o ./Core/Src/SwerveBuffer.su ./Core/Src/UART_RM25.cyclo ./Core/Src/UART_RM25.d ./Core/Src/UART_RM25.o ./Core/Src/UART_RM25.su ./Core/Src/bsp_imu_pwm.cyclo ./Core/Src/bsp_imu_pwm.d ./Core/Src/bsp_imu_pwm.o ./Core/Src/bsp_imu_pwm.su ./Core/Src/freertos.cyclo ./Core/Src/freertos.d ./Core/Src/freertos.o ./Core/Src/freertos.su ./Core/Src/imu_temp_control_task.cyclo ./Core/Src/imu_temp_control_task.d ./Core/Src/imu_temp_control_task.o ./Core/Src/imu_temp_control_task.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/pid.cyclo ./Core/Src/pid.d ./Core/Src/pid.o ./Core/Src/pid.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su

.PHONY: clean-Core-2f-Src

