################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/Waveshare_10Dof-D.c \
../Core/Src/fatfs_sd.c \
../Core/Src/i2c.c \
../Core/Src/main.c \
../Core/Src/mpu9255.c \
../Core/Src/mpu925x_core.c \
../Core/Src/mpu925x_internals.c \
../Core/Src/mpu925x_settings.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c 

C_DEPS += \
./Core/Src/Waveshare_10Dof-D.d \
./Core/Src/fatfs_sd.d \
./Core/Src/i2c.d \
./Core/Src/main.d \
./Core/Src/mpu9255.d \
./Core/Src/mpu925x_core.d \
./Core/Src/mpu925x_internals.d \
./Core/Src/mpu925x_settings.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d 

OBJS += \
./Core/Src/Waveshare_10Dof-D.o \
./Core/Src/fatfs_sd.o \
./Core/Src/i2c.o \
./Core/Src/main.o \
./Core/Src/mpu9255.o \
./Core/Src/mpu925x_core.o \
./Core/Src/mpu925x_internals.o \
./Core/Src/mpu925x_settings.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../FATFS/Target -I../FATFS/App -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FatFs/src -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/mshahsavar/STM32CubeIDE/workspace_1.9.0/Waveshare1/Drivers/SSD1306" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/Waveshare_10Dof-D.d ./Core/Src/Waveshare_10Dof-D.o ./Core/Src/Waveshare_10Dof-D.su ./Core/Src/fatfs_sd.d ./Core/Src/fatfs_sd.o ./Core/Src/fatfs_sd.su ./Core/Src/i2c.d ./Core/Src/i2c.o ./Core/Src/i2c.su ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/mpu9255.d ./Core/Src/mpu9255.o ./Core/Src/mpu9255.su ./Core/Src/mpu925x_core.d ./Core/Src/mpu925x_core.o ./Core/Src/mpu925x_core.su ./Core/Src/mpu925x_internals.d ./Core/Src/mpu925x_internals.o ./Core/Src/mpu925x_internals.su ./Core/Src/mpu925x_settings.d ./Core/Src/mpu925x_settings.o ./Core/Src/mpu925x_settings.su ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su

.PHONY: clean-Core-2f-Src

