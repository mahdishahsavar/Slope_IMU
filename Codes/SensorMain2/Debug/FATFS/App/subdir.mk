################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../FATFS/App/fatfs.c 

C_DEPS += \
./FATFS/App/fatfs.d 

OBJS += \
./FATFS/App/fatfs.o 


# Each subdirectory must supply rules for building sources it contributes
FATFS/App/%.o FATFS/App/%.su: ../FATFS/App/%.c FATFS/App/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -IC:/Users/mshahsavar/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.0/Drivers/STM32F4xx_HAL_Driver/Inc -IC:/Users/mshahsavar/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.0/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -IC:/Users/mshahsavar/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.0/Drivers/CMSIS/Device/ST/STM32F4xx/Include -IC:/Users/mshahsavar/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.0/Drivers/CMSIS/Include -I"C:/Users/mshahsavar/STM32CubeIDE/workspace_1.9.0/SensorMain2/Drivers/SSD1306" -I../FATFS/Target -I../FATFS/App -IC:/Users/mshahsavar/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.0/Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-FATFS-2f-App

clean-FATFS-2f-App:
	-$(RM) ./FATFS/App/fatfs.d ./FATFS/App/fatfs.o ./FATFS/App/fatfs.su

.PHONY: clean-FATFS-2f-App

