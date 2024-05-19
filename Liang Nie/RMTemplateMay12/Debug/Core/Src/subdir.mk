################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/BMI088Middleware.c \
../Core/Src/BMI088driver.c \
../Core/Src/CAN.c \
../Core/Src/CAN_receive.c \
../Core/Src/UART.c \
../Core/Src/bsp_can.c \
../Core/Src/bsp_imu_pwm.c \
../Core/Src/bsp_rc.c \
../Core/Src/freertos.c \
../Core/Src/imu_temp_control_task.c \
../Core/Src/ist8310driver.c \
../Core/Src/ist8310driver_middleware.c \
../Core/Src/main.c \
../Core/Src/pid.c \
../Core/Src/pwm.c \
../Core/Src/remote_control.c \
../Core/Src/songs.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c 

OBJS += \
./Core/Src/BMI088Middleware.o \
./Core/Src/BMI088driver.o \
./Core/Src/CAN.o \
./Core/Src/CAN_receive.o \
./Core/Src/UART.o \
./Core/Src/bsp_can.o \
./Core/Src/bsp_imu_pwm.o \
./Core/Src/bsp_rc.o \
./Core/Src/freertos.o \
./Core/Src/imu_temp_control_task.o \
./Core/Src/ist8310driver.o \
./Core/Src/ist8310driver_middleware.o \
./Core/Src/main.o \
./Core/Src/pid.o \
./Core/Src/pwm.o \
./Core/Src/remote_control.o \
./Core/Src/songs.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o 

C_DEPS += \
./Core/Src/BMI088Middleware.d \
./Core/Src/BMI088driver.d \
./Core/Src/CAN.d \
./Core/Src/CAN_receive.d \
./Core/Src/UART.d \
./Core/Src/bsp_can.d \
./Core/Src/bsp_imu_pwm.d \
./Core/Src/bsp_rc.d \
./Core/Src/freertos.d \
./Core/Src/imu_temp_control_task.d \
./Core/Src/ist8310driver.d \
./Core/Src/ist8310driver_middleware.d \
./Core/Src/main.d \
./Core/Src/pid.d \
./Core/Src/pwm.d \
./Core/Src/remote_control.d \
./Core/Src/songs.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/BMI088Middleware.cyclo ./Core/Src/BMI088Middleware.d ./Core/Src/BMI088Middleware.o ./Core/Src/BMI088Middleware.su ./Core/Src/BMI088driver.cyclo ./Core/Src/BMI088driver.d ./Core/Src/BMI088driver.o ./Core/Src/BMI088driver.su ./Core/Src/CAN.cyclo ./Core/Src/CAN.d ./Core/Src/CAN.o ./Core/Src/CAN.su ./Core/Src/CAN_receive.cyclo ./Core/Src/CAN_receive.d ./Core/Src/CAN_receive.o ./Core/Src/CAN_receive.su ./Core/Src/UART.cyclo ./Core/Src/UART.d ./Core/Src/UART.o ./Core/Src/UART.su ./Core/Src/bsp_can.cyclo ./Core/Src/bsp_can.d ./Core/Src/bsp_can.o ./Core/Src/bsp_can.su ./Core/Src/bsp_imu_pwm.cyclo ./Core/Src/bsp_imu_pwm.d ./Core/Src/bsp_imu_pwm.o ./Core/Src/bsp_imu_pwm.su ./Core/Src/bsp_rc.cyclo ./Core/Src/bsp_rc.d ./Core/Src/bsp_rc.o ./Core/Src/bsp_rc.su ./Core/Src/freertos.cyclo ./Core/Src/freertos.d ./Core/Src/freertos.o ./Core/Src/freertos.su ./Core/Src/imu_temp_control_task.cyclo ./Core/Src/imu_temp_control_task.d ./Core/Src/imu_temp_control_task.o ./Core/Src/imu_temp_control_task.su ./Core/Src/ist8310driver.cyclo ./Core/Src/ist8310driver.d ./Core/Src/ist8310driver.o ./Core/Src/ist8310driver.su ./Core/Src/ist8310driver_middleware.cyclo ./Core/Src/ist8310driver_middleware.d ./Core/Src/ist8310driver_middleware.o ./Core/Src/ist8310driver_middleware.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/pid.cyclo ./Core/Src/pid.d ./Core/Src/pid.o ./Core/Src/pid.su ./Core/Src/pwm.cyclo ./Core/Src/pwm.d ./Core/Src/pwm.o ./Core/Src/pwm.su ./Core/Src/remote_control.cyclo ./Core/Src/remote_control.d ./Core/Src/remote_control.o ./Core/Src/remote_control.su ./Core/Src/songs.cyclo ./Core/Src/songs.d ./Core/Src/songs.o ./Core/Src/songs.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su

.PHONY: clean-Core-2f-Src

